use core::error::Error;

use crate::rail::{self, sl_rail_status_t};

#[derive(Debug)]
pub enum RailError {
    InvalidParameter,
    InvalidState,
    InvalidCall,
    SchedulingError,
    PacketBufferEmpty,
    BufferTooSmall(u16),
    TxFifoWriteFail(u16, u16),
    UnknownError(u32),
}

impl core::fmt::Display for RailError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RailError::InvalidParameter => write!(f, "invalid parameters provided"),
            RailError::InvalidState => write!(f, "method called in invalid state"),
            RailError::InvalidCall => write!(f, "invalid call"),
            RailError::SchedulingError => write!(f, "failed to schedule operation"),
            RailError::PacketBufferEmpty => {
                write!(f, "tried to read a packet although the buffer is empty")
            }
            RailError::TxFifoWriteFail(bytes_written, packet_length) => write!(
                f,
                "only wrote {bytes_written} of {packet_length} bytes into the FIFO buffer, possibly it's already full"
            ),
            RailError::UnknownError(error_code) => {
                write!(f, "sl_rail failed with error code {error_code}")
            }
            RailError::BufferTooSmall(packet_size) => write!(
                f,
                "the provided buffer is to small to store the full packet of size {packet_size}"
            ),
        }
    }
}
impl Error for RailError {}

pub type RailResult<T> = Result<T, RailError>;
pub trait IntoRailResult {
    fn into_rail_result(self) -> RailResult<()>;
}
impl IntoRailResult for sl_rail_status_t {
    fn into_rail_result(self) -> RailResult<()> {
        match self {
            rail::SL_RAIL_STATUS_NO_ERROR | rail::SL_RAIL_STATUS_SUSPENDED => Ok(()),
            rail::SL_RAIL_STATUS_INVALID_CALL => Err(RailError::InvalidCall),
            rail::SL_RAIL_STATUS_INVALID_PARAMETER => Err(RailError::InvalidParameter),
            rail::SL_RAIL_STATUS_INVALID_STATE => Err(RailError::InvalidState),
            rail::SL_RAIL_STATUS_SCHED_ERROR => Err(RailError::SchedulingError),
            status => Err(RailError::UnknownError(status)),
        }
    }
}
