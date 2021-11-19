#![macro_use]

use core::future::Future;
use core::sync::atomic::{fence, Ordering};
use core::task::{Poll, Waker};

use embassy::interrupt::{Interrupt, InterruptExt};
use embassy::waitqueue::AtomicWaker;
use embassy_hal_common::drop::OnDrop;
use futures::future::poll_fn;

use crate::dma::{Channel, Request};
use crate::interrupt;
use crate::pac;
use crate::pac::bdma::vals;
use crate::rcc::sealed::RccPeripheral;

const CH_COUNT: usize = pac::peripheral_count!(bdma) * 8;

struct State {
    ch_wakers: [AtomicWaker; CH_COUNT],
}

impl State {
    const fn new() -> Self {
        const AW: AtomicWaker = AtomicWaker::new();
        Self {
            ch_wakers: [AW; CH_COUNT],
        }
    }
}

static STATE: State = State::new();

macro_rules! dma_num {
    (DMA1) => {
        0
    };
    (DMA2) => {
        1
    };
    (BDMA) => {
        0
    };
}

unsafe fn on_irq() {
    pac::peripherals! {
        (bdma, $dma:ident) => {
                let isr = pac::$dma.isr().read();
                let dman = dma_num!($dma);

                for chn in 0..crate::pac::dma_channels_count!($dma) {
                    let cr = pac::$dma.ch(chn).cr();
                    if isr.tcif(chn) && cr.read().tcie() {
                        cr.write(|_| ()); // Disable channel interrupts with the default value.
                        let n = dma_num!($dma) * 8 + chn;
                        STATE.ch_wakers[n].wake();
                    }
                }
        };
    }
}

/// safety: must be called only once
pub(crate) unsafe fn init() {
    pac::interrupts! {
        ($peri:ident, bdma, $block:ident, $signal_name:ident, $irq:ident) => {
            crate::interrupt::$irq::steal().enable();
        };
    }
    pac::peripherals! {
        (bdma, $peri:ident) => {
            crate::peripherals::$peri::enable();
        };
    }
}

macro_rules! impl_do_transfer {
    ($dma_peri:ident, $channel_num:expr, $request:expr, $peri_addr:expr, $buf:expr, $count:expr, $incr_mem:expr, $dir:expr, $size:expr) => {
        unsafe {
            low_level_api::do_transfer(
                crate::pac::$dma_peri,
                $channel_num,
                (dma_num!($dma_peri) * 8) + $channel_num,
                $request,
                $dir,
                $peri_addr as *const u32,
                $buf,
                $count,
                $incr_mem,
                $size,
                #[cfg(dmamux)]
                <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                #[cfg(dmamux)]
                <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
            )
        }
    };
}

macro_rules! impl_start_transfer {
    ($dma_peri:ident, $channel_num:expr, $request:expr, $peri_addr:expr, $buf:expr, $dir:expr, $size:expr) => {
        unsafe {
            low_level_api::reset_status(crate::pac::$dma_peri, $channel_num);
            low_level_api::start_transfer(
                crate::pac::$dma_peri,
                $channel_num,
                #[cfg(any(bdma_v2, dmamux))]
                $request,
                $dir,
                $peri_addr as *const u32,
                $buf.as_ptr() as *mut u32,
                $buf.len(),
                true,
                $size,
                #[cfg(dmamux)]
                <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                #[cfg(dmamux)]
                <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
            )
        }
    };
}

pac::dma_channels! {
    ($channel_peri:ident, $dma_peri:ident, bdma, $channel_num:expr, $dmamux:tt) => {
        impl crate::dma::sealed::Channel for crate::peripherals::$channel_peri {}

        impl Channel for crate::peripherals::$channel_peri
        {
            type ReadFuture<'a> = impl Future<Output = ()> + 'a;
            type WriteFuture<'a> = impl Future<Output = ()> + 'a;
            type CompletionFuture<'a> = impl Future<Output = ()> + 'a;

            fn read_u8<'a>(
                &'a mut self,
                request: Request,
                src: *mut u32,
                buf: &'a mut [u8],
            ) -> Self::ReadFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, src, buf.as_mut_ptr() as *mut u32, buf.len(), true, vals::Dir::FROMPERIPHERAL, vals::Size::BITS8)
            }

            fn read_u16<'a>(
                &'a mut self,
                request: Request,
                reg_addr: *mut u32,
                buf: &'a mut [u16],
            ) -> Self::ReadFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_mut_ptr() as *mut u32, buf.len(), true, vals::Dir::FROMPERIPHERAL, vals::Size::BITS16)
            }

            fn read_u32<'a>(
                &'a mut self,
                request: Request,
                reg_addr: *mut u32,
                buf: &'a mut [u32],
            ) -> Self::ReadFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_mut_ptr() as *mut u32, buf.len(), true, vals::Dir::FROMPERIPHERAL, vals::Size::BITS32)
            }

            fn write_u8<'a>(
                &'a mut self,
                request: Request,
                buf: &'a [u8],
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_ptr() as *mut u32, buf.len(), true, vals::Dir::FROMMEMORY, vals::Size::BITS8)
            }

            fn write_u16<'a>(
                &'a mut self,
                request: Request,
                buf: &'a [u16],
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_ptr() as *mut u32, buf.len(), true, vals::Dir::FROMMEMORY, vals::Size::BITS16)
            }

            fn write_u32<'a>(
                &'a mut self,
                request: Request,
                buf: &'a [u32],
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_ptr() as *mut u32, buf.len(), true, vals::Dir::FROMMEMORY, vals::Size::BITS32)
            }

            fn write_x<'a>(
                &'a mut self,
                request: Request,
                word: &u8,
                count: usize,
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr,
                    word as *const u8 as *mut u32, count, false, vals::Dir::FROMMEMORY, vals::Size::BITS32)
            }

            fn start_write_u8<'a>(&'a mut self, request: Request, buf: &'a [u8], reg_addr: *mut u32){
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::FROMMEMORY, vals::Size::BITS8)
            }

            fn start_write_u16<'a>(&'a mut self, request: Request, buf: &'a [u16], reg_addr: *mut u32){
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::FROMMEMORY, vals::Size::BITS16)
            }

            fn start_write_u32<'a>(&'a mut self, request: Request, buf: &'a [u32], reg_addr: *mut u32){
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::FROMMEMORY, vals::Size::BITS32)
            }

            fn start_read_u8<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u8]){
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::FROMPERIPHERAL, vals::Size::BITS8)
            }

            fn start_read_u16<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u16]){
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::FROMPERIPHERAL, vals::Size::BITS16)
            }

            fn start_read_u32<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u32]){
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::FROMPERIPHERAL, vals::Size::BITS32)
            }

            fn stop<'a>(&'a mut self){
                unsafe {low_level_api::stop(crate::pac::$dma_peri, $channel_num);}
            }

            fn is_stopped<'a>(&'a self) -> bool {
                unsafe {low_level_api::is_stopped(crate::pac::$dma_peri, $channel_num)}
            }
            fn remaining_transfers<'a>(&'a mut self) -> u16 {
                unsafe {low_level_api::get_remaining_transfers(crate::pac::$dma_peri, $channel_num)}
            }

            fn set_waker<'a>(&'a mut self, waker: &'a Waker) {
                unsafe {low_level_api::set_waker(crate::pac::$dma_peri,  $channel_num, waker )}
            }

            fn wait_for_completion<'a>(&mut self) -> Self::CompletionFuture<'a> {
                async move {}
                // unsafe {low_level_api::wait_for_completion(&crate::pac::$dma_peri, (dma_num!($dma_peri) * 8) + $channel_num, $channel_num)}
            }
        }
    };
}

pac::interrupts! {
    ($peri:ident, bdma, $block:ident, $signal_name:ident, $irq:ident) => {
        #[crate::interrupt]
        unsafe fn $irq () {
            on_irq()
        }
    };
}

mod low_level_api {
    use super::*;

    #[allow(unused)]
    pub(crate) unsafe fn do_transfer(
        dma: pac::bdma::Dma,
        channel_number: u8,
        state_number: u8,
        request: Request,
        dir: vals::Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: vals::Size,
        #[cfg(dmamux)] dmamux_regs: pac::dmamux::Dmamux,
        #[cfg(dmamux)] dmamux_ch_num: u8,
    ) -> impl Future<Output = ()> {
        // ndtr is max 16 bits.
        assert!(mem_len <= 0xFFFF);

        // Reset status
        reset_status(dma, channel_number);

        let on_drop = OnDrop::new(move || unsafe {
            stop(dma, channel_number);
        });

        low_level_api::start_transfer(
            dma,
            channel_number,
            #[cfg(any(bdma_v2, dmamux))]
            request,
            dir,
            peri_addr,
            mem_addr,
            mem_len,
            incr_mem,
            data_size,
            #[cfg(dmamux)]
            dmamux_regs,
            #[cfg(dmamux)]
            dmamux_ch_num,
        );

        async move {
            let res = low_level_api::wait_for_completion(dma, state_number, channel_number).await;

            drop(on_drop)
        }
    }

    pub unsafe fn start_transfer(
        dma: pac::bdma::Dma,
        channel_number: u8,
        #[cfg(any(bdma_v2, dmamux))] request: Request,
        dir: vals::Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: vals::Size,
        #[cfg(dmamux)] dmamux_regs: pac::dmamux::Dmamux,
        #[cfg(dmamux)] dmamux_ch_num: u8,
    ) {
        let ch = dma.ch(channel_number as _);

        #[cfg(dmamux)]
        super::super::dmamux::configure_dmamux(dmamux_regs, dmamux_ch_num, request);

        #[cfg(bdma_v2)]
        critical_section::with(|_| {
            dma.cselr()
                .modify(|w| w.set_cs(channel_number as _, request))
        });

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::Release);

        ch.par().write_value(peri_addr as u32);
        ch.mar().write_value(mem_addr as u32);
        ch.ndtr().write(|w| w.set_ndt(mem_len as u16));
        ch.cr().write(|w| {
            w.set_psize(data_size);
            w.set_msize(data_size);
            if incr_mem {
                w.set_minc(vals::Inc::ENABLED);
            } else {
                w.set_minc(vals::Inc::DISABLED);
            }
            w.set_dir(dir);
            w.set_teie(true);
            w.set_tcie(true);
            w.set_en(true);
        });
    }

    pub unsafe fn stop(dma: pac::bdma::Dma, ch: u8) {
        let ch = dma.ch(ch as _);

        // Disable the channel and interrupts with the default value.
        ch.cr().write(|_| ());

        // Wait for the transfer to complete when it was ongoing.
        while ch.cr().read().en() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::Acquire);
    }

    pub unsafe fn is_stopped(dma: pac::bdma::Dma, ch: u8) -> bool {
        let ch = dma.ch(ch as _);
        ch.cr().read().en()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub unsafe fn get_remaining_transfers(dma: pac::bdma::Dma, ch: u8) -> u16 {
        // get a handle on the channel itself
        let ch = dma.ch(ch as _);
        // read the remaining transfer count. If this is zero, the transfer completed fully.
        ch.ndtr().read().ndt()
    }

    /// Sets the waker for the specified DMA channel
    pub unsafe fn set_waker(_dma: pac::bdma::Dma, state_number: u8, waker: &Waker) {
        let n = state_number as usize;
        STATE.ch_wakers[n].register(waker);
    }

    pub unsafe fn reset_status(dma: pac::bdma::Dma, channel_number: u8) {
        dma.ifcr().write(|w| {
            w.set_tcif(channel_number as _, true);
            w.set_teif(channel_number as _, true);
        });
    }

    pub unsafe fn wait_for_completion<'a>(
        dma: crate::pac::bdma::Dma,
        state_number: u8,
        channel_number: u8,
    ) -> impl Future<Output = ()> + 'a {
        poll_fn(move |cx| {
            STATE.ch_wakers[state_number as usize].register(cx.waker());

            let isr = dma.isr().read();

            // TODO handle error
            assert!(!isr.teif(channel_number as _));

            if isr.tcif(channel_number as _) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    }
}
