pub const POS_X: usize = 0;
pub const POS_Y: usize = 1;
pub const POS_Z: usize = 2;
pub const ROLL: usize = 3;
pub const PITCH: usize = 4;
pub const YAW: usize = 5;
pub const VEL_X: usize = 6;
pub const VEL_Y: usize = 7;
pub const VEL_Z: usize = 8;
pub const ROLL_RATE: usize = 9;
pub const PITCH_RATE: usize = 10;
pub const YAW_RATE: usize = 11;
pub const RUDDER_STATE: usize = 12;
pub const SAIL_STATE: usize = 13;

pub const N_STATES: usize = 12;
pub const N_STATES_ACTUATED: usize = 14;

pub type State = nalgebra::SVector<f64, N_STATES_ACTUATED>;
