use std::f32::consts::{PI, TAU};

const DEG_TO_RAD: f32 = PI / 180.;
const BUFFER: f32 = 0.01;
const MAX_ITERATIONS: usize = 100;

pub struct Arm {
    segments: Vec<(f32, f32)>,
}

impl Arm {
    pub fn new<I>(segments: I) -> Self
    where
        I: IntoIterator<Item = f32>,
    {
        Self {
            segments: segments.into_iter().map(|l| (0.0, l)).collect(),
        }
    }

    pub fn reset(&mut self) {
        for (theta, _) in self.segments.iter_mut() {
            *theta = 0.;
        }
    }

    pub fn segments(&self) -> &Vec<(f32, f32)> {
        &self.segments
    }

    pub fn segments_degrees(&self) -> Vec<(f32, f32)> {
        self.segments.iter().map(|(rad, length)| (rad / PI * 180., *length)).collect()
    }

	pub fn push_segment(&mut self, length: f32) {
		self.segments.push((0., length));
	}

	pub fn pop_segment(&mut self) {
		self.segments.pop();
	}

    pub fn forward_kinematics(&self) -> Vec<(f32, f32)> {
        let mut positions = vec![(0., 0.)];
        let mut theta = 0.0;
        let mut x = 0.0;
        let mut y = 0.0;

        for (pivot, length) in self.segments.iter() {
            theta += pivot;
            x += theta.cos() * length;
            y += theta.sin() * length;

            positions.push((x, y));
        }

        positions
    }

    pub fn inverse_kinematics(&mut self, x_f: f32, y_f: f32) {
        for _ in 0..MAX_ITERATIONS {
            for i in 0..self.segments.len() {
                let positions = self.forward_kinematics();

                // position of current segment
                let (x_n, y_n) = positions[i];
                // position of end effector
                let (x_l, y_l) = positions.last().unwrap();

                if f32::hypot(x_f - x_l, y_f - y_l) < BUFFER {
                    return;
                }

                // compute the angle of the triangle created between
                // the current segment point, the last segment point,
                // and the desired end position
                let a = f32::hypot(x_l - x_n, y_l - y_n);
                let b = f32::hypot(x_f - x_n, y_f - y_n);
                let dot = (x_l - x_n) * (x_f - x_n) + (y_l - y_n) * (y_f - y_n);
                let delta = (dot / (a * b)).acos();

                // calculate whether delta calculates for the necessary positive
                // or negative offset to the current segment angle using its normal
                let direction = (y_l - y_n) * (x_f - x_n) - (x_l - x_n) * (y_f - y_n);

                if delta.abs() > f32::EPSILON {
                    if direction.is_sign_positive() {
                        self.segments[i].0 -= delta;
                    } else {
                        self.segments[i].0 += delta;
                    }
                }
            }
        }
    }
}
