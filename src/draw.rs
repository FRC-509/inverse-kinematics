use nannou::prelude::*;

use crate::kinematics::Arm;

const WIDTH: u32 = 500;
const HEIGHT: u32 = 500;

pub fn run_sketch() {
    nannou::app(model).update(update).run()
}

struct Model {
    kinematic_arm: Arm<3>,
    target: (f32, f32),
    mouse_held: bool,
}

fn model(app: &App) -> Model {
    app.new_window()
        .title("Inverse Kinematics")
        .size(WIDTH, HEIGHT)
        .resizable(true)
        .view(view)
        .mouse_pressed(draw_press)
        .mouse_released(draw_released)
        .build()
        .unwrap();

    Model {
        kinematic_arm: Arm::new([50., 50., 100.]),
        target: (0., 0.),
        mouse_held: false,
    }
}

fn update(app: &App, model: &mut Model, _: Update) {
    if model.mouse_held {
        model.target = (app.mouse.x, app.mouse.y);
    }

    model
        .kinematic_arm
        .inverse_kinematics(model.target.0, model.target.1);
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();

    draw.background().color(WHITESMOKE);

    let mut x_offset = 0.;
    let mut y_offset = 0.;
    let mut theta_sum = 0.;

    for (theta, length) in model.kinematic_arm.segments() {
        theta_sum += theta;

        let x_projection = length * theta_sum.cos();
        let y_projection = length * theta_sum.sin();

        let segment_rect = Rect::from_w_h(*length, 25.)
            .middle_of(app.window_rect())
            .shift_x(x_offset + x_projection / 2.)
            .shift_y(y_offset + y_projection / 2.);

        draw.rect()
            .rotate(theta_sum)
            .wh(segment_rect.wh())
            .xy(segment_rect.xy())
            .color(rgba(0., 0., 0., 255.));

        let joint_rect = Rect::from_w_h(*length, 25.)
            .middle_of(app.window_rect())
            .shift_x(x_offset)
            .shift_y(y_offset);

        draw.ellipse()
            .color(YELLOW)
            .w_h(35., 35.)
            .x_y(joint_rect.x(), joint_rect.y());

        x_offset += x_projection;
        y_offset += y_projection;
    }

    draw.ellipse().color(BLACK).w_h(50., 50.).x_y(0., 0.);

    draw.ellipse()
        .color(RED)
        .w_h(50., 50.)
        .x_y(model.target.0, model.target.1);

    draw.to_frame(app, &frame).unwrap();
}

fn draw_press(_: &App, model: &mut Model, mouse: MouseButton) {
    match mouse {
        MouseButton::Left | MouseButton::Right => model.mouse_held = true,
        _ => (),
    }
}

fn draw_released(_: &App, model: &mut Model, mouse: MouseButton) {
    match mouse {
        MouseButton::Left | MouseButton::Right => model.mouse_held = false,
        _ => (),
    }
}
