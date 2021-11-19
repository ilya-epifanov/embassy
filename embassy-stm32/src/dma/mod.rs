#[cfg(bdma)]
mod bdma;
#[cfg(dma)]
mod dma;
#[cfg(dmamux)]
mod dmamux;

#[cfg(dmamux)]
pub use dmamux::*;

use core::future::Future;
use core::task::Waker;
use embassy::util::Unborrow;

#[cfg(any(bdma_v2, dma_v2, dmamux))]
pub type Request = u8;
#[cfg(not(any(bdma_v2, dma_v2, dmamux)))]
pub type Request = ();

pub(crate) mod sealed {
    pub trait Channel {}
}

pub trait Channel: sealed::Channel {
    type ReadFuture<'a>: Future<Output = ()> + 'a
    where
        Self: 'a;

    type WriteFuture<'a>: Future<Output = ()> + 'a
    where
        Self: 'a;

    type CompletionFuture<'a>: Future<Output = ()> + 'a
    where
        Self: 'a;

    fn read_u8<'a>(
        &'a mut self,
        request: Request,
        reg_addr: *mut u32,
        buf: &'a mut [u8],
    ) -> Self::ReadFuture<'a>;

    fn read_u16<'a>(
        &'a mut self,
        request: Request,
        reg_addr: *mut u32,
        buf: &'a mut [u16],
    ) -> Self::ReadFuture<'a>;

    fn read_u32<'a>(
        &'a mut self,
        request: Request,
        reg_addr: *mut u32,
        buf: &'a mut [u32],
    ) -> Self::ReadFuture<'a>;

    fn write_u8<'a>(
        &'a mut self,
        request: Request,
        buf: &'a [u8],
        reg_addr: *mut u32,
    ) -> Self::WriteFuture<'a>;

    fn write_u16<'a>(
        &'a mut self,
        request: Request,
        buf: &'a [u16],
        reg_addr: *mut u32,
    ) -> Self::WriteFuture<'a>;

    fn write_u32<'a>(
        &'a mut self,
        request: Request,
        buf: &'a [u32],
        reg_addr: *mut u32,
    ) -> Self::WriteFuture<'a>;

    fn write_x<'a>(
        &'a mut self,
        request: Request,
        word: &u8,
        num: usize,
        reg_addr: *mut u32,
    ) -> Self::WriteFuture<'a>;

    /// Starts this channel for writing a stream of bytes.
    fn start_write_u8<'a>(&'a mut self, request: Request, buf: &'a [u8], reg_addr: *mut u32);

    /// Starts this channel for writing a stream of half-words.
    fn start_write_u16<'a>(&'a mut self, request: Request, buf: &'a [u16], reg_addr: *mut u32);

    /// Starts this channel for writing a stream of words.
    fn start_write_u32<'a>(&'a mut self, request: Request, buf: &'a [u32], reg_addr: *mut u32);

    /// Starts this channel for reading a stream of bytes.
    fn start_read_u8<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u8]);

    /// Starts this channel for reading a stream of half-words.
    fn start_read_u16<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u16]);

    /// Starts this channel for reading a stream of words.
    fn start_read_u32<'a>(&'a mut self, request: Request, reg_addr: *mut u32, buf: &'a mut [u32]);

    /// Stops this channel.
    fn stop<'a>(&'a mut self);

    /// Returns whether this channel is active or stopped.
    fn is_stopped<'a>(&self) -> bool;

    /// Returns the total number of remaining transfers.
    fn remaining_transfers<'a>(&'a mut self) -> u16;

    /// Sets the waker that is called when this channel completes/
    fn set_waker(&mut self, waker: &Waker);

    fn wait_for_completion<'a>(&mut self) -> Self::CompletionFuture<'a>;
}

pub struct NoDma;

unsafe impl Unborrow for NoDma {
    type Target = NoDma;

    unsafe fn unborrow(self) -> Self::Target {
        self
    }
}

// safety: must be called only once at startup
pub(crate) unsafe fn init() {
    #[cfg(bdma)]
    bdma::init();
    #[cfg(dma)]
    dma::init();
    #[cfg(dmamux)]
    dmamux::init();
}
