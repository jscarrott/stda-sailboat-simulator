use polars::prelude::*;

fn main() {
    println!("Hello, world!");
    run_scenario(Scenario {
        n_states: 12,
        actor_dynamics: false,
        wind: Truewind {
            x: 0f32,
            y: 0f32,
            angle: 5f32,
            speed: 10f32,
        },
    })
}

struct Truewind {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub speed: f32,
}

struct ApparentWind {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub speed: f32,
}

struct Scenario {
    pub n_states: u32,
    pub actor_dynamics: bool,
    pub wind: Truewind,
}

struct Environment {
    pub sail_angle: f32,
    pub rudder_angle: f32,
    pub true_wind: Truewind,
    pub wave: Option<f32>, //todo
}

// struct X0 {
//     pub x: f32,
//     pub r, f32,
//     pub sail: f32,
//     pub t: f32,
//     pub x: f32,
// }

fn run_scenario(scenario: Scenario) {
    let mut n_states = 12;
    if scenario.actor_dynamics {
        n_states = 14;
    }
    let wind = Truewind {
        x: 0f32,
        y: 5f32,
        angle: 5f32,
        speed: std::f32::consts::PI,
    };
    let pi = std::f32::consts::PI;
    let mut environment = Environment {
        sail_angle: 0f32,
        rudder_angle: 0f32,
        true_wind: wind,
        wave: None,
    };
    // Not done wave
    let end_time = 150f32;
    let sample_time: f32 = 0.3;
    let sail_sample_time = 2f32;
    let number_of_steps: u32 = (end_time / sample_time) as u32;
    let mut x0 = Series::new("x0", vec![0f32; n_states]);

    let x = init_dataframe(n_states as usize, number_of_steps as usize);
    x.select_physical() = x0.clone();
    let r = init_dataframe(n_states as usize, number_of_steps as usize);
    let sail = init_dataframe(n_states as usize, number_of_steps as usize);
    let t = init_dataframe(n_states as usize, number_of_steps as usize);
    let ref_heading = init_dataframe(n_states as usize, number_of_steps as usize);

    println!("{x}");
}

fn init_dataframe(n_state: usize, number_of_steps: usize) -> DataFrame {
    let mut init_seq = vec![];
    for x in 1..number_of_steps {
        let mut input = vec![0f32; n_state];
        let mut series = Series::new(&format!("{x}"), input);
        // series.fill_null(FillNullStrategy::Zero);
        init_seq.push(series);
    }

    DataFrame::new(init_seq).unwrap()
}
