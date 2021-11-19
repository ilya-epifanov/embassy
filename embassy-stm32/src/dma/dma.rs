use core::future::Future;
use core::sync::atomic::{fence, Ordering};
use core::task::{Poll, Waker};

use embassy::interrupt::{Interrupt, InterruptExt};
use embassy::waitqueue::AtomicWaker;
use embassy_hal_common::drop::OnDrop;
use futures::future::poll_fn;

use crate::interrupt;
use crate::pac;
use crate::pac::dma::{regs, vals};
use crate::rcc::sealed::RccPeripheral;

use super::{Channel, Request};

const CH_COUNT: usize = pac::peripheral_count!(DMA) * 8;

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
}

unsafe fn on_irq() {
    pac::peripherals! {
        (dma, $dma:ident) => {
            for isrn in 0..2 {
                let isr = pac::$dma.isr(isrn).read();

                for chn in 0..4 {
                    let cr = pac::$dma.st(isrn * 4 + chn).cr();

                    if isr.tcif(chn) && cr.read().tcie() {
                        cr.write(|_| ()); // Disable channel interrupts with the default value.
                        let n = dma_num!($dma) * 8 + isrn * 4 + chn;
                        STATE.ch_wakers[n].wake();
                    }
                }
            }
        };
    }
}

/// safety: must be called only once
pub(crate) unsafe fn init() {
    pac::interrupts! {
        ($peri:ident, dma, $block:ident, $signal_name:ident, $irq:ident) => {
            interrupt::$irq::steal().enable();
        };
    }
    pac::peripherals! {
        (dma, $peri:ident) => {
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
            let isrn = $channel_num as usize / 4;
            let isrbit = $channel_num as usize % 4;
            low_level_api::reset_status(&crate::pac::$dma_peri, isrn, isrbit);
            low_level_api::start_transfer(
                $request,
                $dir,
                $peri_addr as *const u32,
                $buf.as_ptr() as *mut u32,
                $buf.len(),
                true,
                crate::pac::$dma_peri.st($channel_num as _),
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
    ($channel_peri:ident, $dma_peri:ident, dma, $channel_num:expr, $dmamux:tt) => {
        impl crate::dma::sealed::Channel for crate::peripherals::$channel_peri {}

        impl Channel for crate::peripherals::$channel_peri {
            type ReadFuture<'a> = impl Future<Output = ()> + 'a;
            type WriteFuture<'a> = impl Future<Output = ()> + 'a;
            type CompletionFuture<'a> = impl Future<Output = ()> + 'a;

            fn read_u8<'a>(
                &'a mut self,
                request: Request,
                reg_addr: *mut u32,
                buf: &'a mut [u8],
            ) -> Self::ReadFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_mut_ptr() as *mut u32, buf.len(), true, vals::Dir::PERIPHERALTOMEMORY, vals::Size::BITS8)
            }

            fn read_u16<'a>(
                &'a mut self,
                request: Request,
                reg_addr: *mut u32,
                buf: &'a mut [u16],
            ) -> Self::ReadFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_mut_ptr() as *mut u32, buf.len(), true, vals::Dir::PERIPHERALTOMEMORY, vals::Size::BITS16)
            }

            fn read_u32<'a>(
                &'a mut self,
                request: Request,
                reg_addr: *mut u32,
                buf: &'a mut [u32],
            ) -> Self::ReadFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_mut_ptr() as *mut u32, buf.len(), true, vals::Dir::PERIPHERALTOMEMORY, vals::Size::BITS32)
            }

            fn write_u8<'a>(
                &'a mut self,
                request: Request,
                buf: &'a [u8],
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_ptr() as *mut u32, buf.len(), true, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS8)
            }

            fn write_u16<'a>(
                &'a mut self,
                request: Request,
                buf: &'a [u16],
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_ptr() as *mut u32, buf.len(), true, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS16)
            }

            fn write_u32<'a>(
                &'a mut self,
                request: Request,
                buf: &'a [u32],
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, buf.as_ptr() as *mut u32, buf.len(), true, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS32)
            }

            fn write_x<'a>(
                &'a mut self,
                request: Request,
                word: &u8,
                num: usize,
                reg_addr: *mut u32,
            ) -> Self::WriteFuture<'a> {
                impl_do_transfer!($dma_peri, $channel_num, request, reg_addr, word as *const u8 as *mut u32, num, false, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS8)
            }

            fn start_write_u8<'a>(&'a mut self, request: Request, buf: &'a [u8], reg_addr: *mut u32) {
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS8)
            }

            fn start_write_u16<'a>(&'a mut self, request: Request, buf: &'a [u16], reg_addr: *mut u32) {
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS16)
            }

            fn start_write_u32<'a>(&'a mut self, request: Request, buf: &'a [u32], reg_addr: *mut u32) {
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::MEMORYTOPERIPHERAL, vals::Size::BITS32)
            }

            fn start_read_u8<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u8]) {
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::PERIPHERALTOMEMORY, vals::Size::BITS8)
            }

            fn start_read_u16<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u16]) {
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::PERIPHERALTOMEMORY, vals::Size::BITS16)
            }

            fn start_read_u32<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u32]) {
                impl_start_transfer!($dma_peri, $channel_num, request, reg_addr, buf, vals::Dir::PERIPHERALTOMEMORY, vals::Size::BITS32)
            }

            fn stop<'a>(&'a mut self) {
                unsafe {low_level_api::stop(&crate::pac::$dma_peri, $channel_num);}
            }

            fn is_stopped<'a>(&'a self) -> bool {
                unsafe {low_level_api::is_stopped(&crate::pac::$dma_peri, $channel_num)}
            }

            fn remaining_transfers<'a>(&'a mut self) -> u16 {
                unsafe {low_level_api::get_remaining_transfers(&crate::pac::$dma_peri, $channel_num)}
            }

            fn set_waker<'a>(&'a mut self, waker: &'a Waker) {
                unsafe {low_level_api::set_waker(&crate::pac::$dma_peri,  $channel_num, waker )}
            }

            fn wait_for_completion<'a>(&mut self) -> Self::CompletionFuture<'a> {
                unsafe {low_level_api::wait_for_completion(&crate::pac::$dma_peri, (dma_num!($dma_peri) * 8) + $channel_num, $channel_num)}
            }
        }
    };
}

pac::interrupts! {
    ($peri:ident, dma, $block:ident, $signal_name:ident, $irq:ident) => {
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
        dma: pac::dma::Dma,
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
        let isrn = channel_number as usize / 4;
        let isrbit = channel_number as usize % 4;
        reset_status(&dma, isrn, isrbit);

        let ch = dma.st(channel_number as _);

        let on_drop = OnDrop::new(move || unsafe {
            low_level_api::stop(&dma, channel_number);
        });

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::Release);

        // Actually start the transaction
        start_transfer(
            request,
            dir,
            peri_addr,
            mem_addr,
            mem_len,
            incr_mem,
            ch,
            data_size,
            #[cfg(dmamux)]
            dmamux_regs,
            #[cfg(dmamux)]
            dmamux_ch_num,
        );

        async move {
            let res = wait_for_completion(&dma, state_number, channel_number).await;

            drop(on_drop)
        }
    }

    pub unsafe fn start_transfer(
        request: Request,
        dir: vals::Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        ch: crate::pac::dma::St,
        data_size: vals::Size,
        #[cfg(dmamux)] dmamux_regs: pac::dmamux::Dmamux,
        #[cfg(dmamux)] dmamux_ch_num: u8,
    ) {
        #[cfg(dmamux)]
        super::super::dmamux::configure_dmamux(dmamux_regs, dmamux_ch_num, request);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::Release);

        ch.par().write_value(peri_addr as u32);
        ch.m0ar().write_value(mem_addr as u32);
        ch.ndtr().write_value(regs::Ndtr(mem_len as _));
        ch.cr().write(|w| {
            w.set_dir(dir);
            w.set_msize(data_size);
            w.set_psize(data_size);
            w.set_pl(vals::Pl::VERYHIGH);
            if incr_mem {
                w.set_minc(vals::Inc::INCREMENTED);
            } else {
                w.set_minc(vals::Inc::FIXED);
            }
            w.set_pinc(vals::Inc::FIXED);
            w.set_teie(true);
            w.set_tcie(true);
            #[cfg(dma_v1)]
            w.set_trbuff(true);

            #[cfg(dma_v2)]
            w.set_chsel(request);

            w.set_en(true);
        });
    }

    /// Stops the DMA channel.
    pub unsafe fn stop(dma: &pac::dma::Dma, ch: u8) {
        // get a handle on the channel itself
        let ch = dma.st(ch as _);

        // Disable the channel and interrupts with the default value.
        ch.cr().write(|_| ());

        // Wait for the transfer to complete when it was ongoing.
        while ch.cr().read().en() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::Acquire);
    }

    /// Gets the running status of the channel
    pub unsafe fn is_stopped(dma: &pac::dma::Dma, ch: u8) -> bool {
        // get a handle on the channel itself
        let ch = dma.st(ch as _);

        // Wait for the transfer to complete when it was ongoing.
        ch.cr().read().en()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub unsafe fn get_remaining_transfers(dma: &pac::dma::Dma, ch: u8) -> u16 {
        // get a handle on the channel itself
        let ch = dma.st(ch as _);
        // read the remaining transfer count. If this is zero, the transfer completed fully.
        ch.ndtr().read().ndt()
    }

    /// Sets the waker for the specified DMA channel
    pub unsafe fn set_waker(_dma: &pac::dma::Dma, state_number: u8, waker: &Waker) {
        let n = state_number as usize;
        STATE.ch_wakers[n].register(waker);
    }

    pub unsafe fn reset_status(dma: &crate::pac::dma::Dma, isrn: usize, isrbit: usize) {
        dma.ifcr(isrn).write(|w| {
            w.set_tcif(isrbit, true);
            w.set_teif(isrbit, true);
        });
    }

    pub unsafe fn wait_for_completion<'a>(
        dma: &'a crate::pac::dma::Dma,
        state_number: u8,
        channel_number: u8,
    ) -> impl Future<Output = ()> + 'a {
        let isrn = channel_number as usize / 4;
        let isrbit = channel_number as usize % 4;

        poll_fn(move |cx| {
            unsafe { set_waker(&dma, state_number, cx.waker()) };

            let isr = dma.isr(isrn).read();

            // TODO handle error
            assert!(!isr.teif(isrbit));

            if isr.tcif(isrbit) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    }
}
