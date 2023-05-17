#![feature(let_chains)]

use std::path::PathBuf;

use frame_capturer::{CaptureMode, FrameCapturer};
use nannou::{prelude::*, wgpu::ToTextureView};

pub type ModelFn<Model> = dyn FnOnce(&App) -> Model;
pub type EventFn<Model> = fn(&App, &mut Model, WindowEvent);
pub type UpdateFn<Model> = fn(&App, &mut Model, Update);
pub type DrawFn<Model> = fn(&App, &Model, &Draw);
pub type ExitFn<Model> = fn(&App, Model);

struct CaptureState {
    space_to_texture: Mat4,
    frame_capturer: FrameCapturer,
    texture_copy: wgpu::Texture,
    texture_copy_view: wgpu::TextureView,
}

struct Model<M> {
    aspect_ratio: f32,
    capture_state: Option<CaptureState>,
    user_model: M,
    events: VisualizationEvents<M>,
}

struct CaptureData {
    directory: PathBuf,
    resolution: (u32, u32),
}

pub struct VisualizationBuilder<M = ()> {
    capture_data: Option<CaptureData>,
    aspect_ratio: f32,
    events: VisualizationEvents<M>,
    model: Box<ModelFn<M>>,
}

#[derive(Clone, Copy)]
struct VisualizationEvents<M = ()> {
    update: Option<UpdateFn<M>>,
    event: Option<EventFn<M>>,
    exit: Option<ExitFn<M>>,
    draw: Option<DrawFn<M>>,
}

impl<M: 'static> VisualizationBuilder<M> {
    pub fn new<F: 'static + FnOnce(&App) -> M>(model: F) -> Self {
        Self {
            model: Box::new(model),
            events: VisualizationEvents {
                update: None,
                draw: None,
                exit: None,
                event: None,
            },
            capture_data: None,
            aspect_ratio: 1.0,
        }
    }

    fn create_model(self, app: &App) -> Model<M> {
        Model {
            aspect_ratio: self.aspect_ratio,
            user_model: (self.model)(app),
            events: self.events,
            capture_state: self.capture_data.map(|capture_data| {
                let texture_size = [capture_data.resolution.0, capture_data.resolution.1];

                let space_to_texture = Mat4::from_translation(
                    (-Vec2::from(texture_size.map(|v| v as f32)) / 2.0).extend(0.0),
                ) * Mat4::from_scale({
                    let scale = texture_size[1] as f32;
                    Vec3::new(scale, scale, scale)
                });

                let texture_copy = nannou::wgpu::TextureBuilder::new()
                    .size(texture_size)
                    .format(nannou::Frame::TEXTURE_FORMAT)
                    .sample_count(1)
                    .usage(
                        wgpu::TextureUsages::RENDER_ATTACHMENT
                            | wgpu::TextureUsages::TEXTURE_BINDING,
                    )
                    .build(app.main_window().device());

                let texture_copy_view = texture_copy.to_texture_view();

                CaptureState {
                    space_to_texture,
                    frame_capturer: FrameCapturer::new(
                        &app.main_window(),
                        texture_size,
                        CaptureMode::Capture {
                            directory: capture_data.directory,
                        },
                    ),
                    texture_copy,
                    texture_copy_view,
                }
            }),
        }
    }

    pub fn update(mut self, update: UpdateFn<M>) -> Self {
        self.events.update = Some(update);
        self
    }

    pub fn event(mut self, event: EventFn<M>) -> Self {
        self.events.event = Some(event);
        self
    }

    pub fn draw(mut self, draw: DrawFn<M>) -> Self {
        self.events.draw = Some(draw);
        self
    }

    pub fn exit(mut self, exit: ExitFn<M>) -> Self {
        self.events.exit = Some(exit);
        self
    }

    pub fn with_aspect_ratio(mut self, aspect_ratio: f32) -> Self {
        self.aspect_ratio = aspect_ratio;
        self
    }

    pub fn with_capture(mut self, directory: PathBuf, resolution: (u32, u32)) -> Self {
        let (width, height) = resolution;
        self.capture_data = Some(CaptureData {
            directory,
            resolution: (width, height),
        });
        self
    }

    pub fn run(self) {
        nannou::app(|app| {
            app.new_window().view(view::<M>).event(event::<M>).build().unwrap();
            self.create_model(app)
        })
        .update(update)
        .exit(exit)
        .run();
    }
}

fn event<M>(app: &App, model: &mut Model<M>, window_event: WindowEvent) {
    if let Some(event) = model.events.event {
        event(app, &mut model.user_model, window_event);
    }
}

fn update<M>(app: &App, model: &mut Model<M>, update_data: Update) {
    if let Some(update) = model.events.update {
        update(app, &mut model.user_model, update_data);
    }
    if let Some(draw) = model.events.draw && let Some(capture_state) = &mut model.capture_state {
        {
            let draw_obj = capture_state.frame_capturer.get_draw();
            draw_obj.reset();

            let draw_obj = &draw_obj.transform(capture_state.space_to_texture);
            draw(app, &model.user_model, draw_obj);
        }

        capture_state
            .frame_capturer
            .render_to_texture(&app.main_window());
    }
}

fn view<M>(app: &App, model: &Model<M>, frame: nannou::Frame) {
    if let Some(capture_state) = &model.capture_state {
        {
            let mut encoder = frame.command_encoder();
            capture_state
                .frame_capturer
                .draw_to_texture(&mut encoder, &capture_state.texture_copy_view);
        }
        let scale = {
            let [w, h] = capture_state.texture_copy.size();
            let w = w as f32;
            let h = h as f32;
            let [win_w, win_h] = frame.texture_size();
            let win_w = win_w as f32;
            let win_h = win_h as f32;
            f32::min(win_w / w, win_h / h)
        };
        let draw = app.draw().scale(scale / app.main_window().scale_factor());
        draw.texture(&capture_state.texture_copy);
        draw.to_frame(app, &frame).unwrap();
    } else if let Some(draw) = model.events.draw {
        let draw_obj = &app.draw();
        let window = app.main_window();
        let scale = {
            let h = 1.0;
            let w = model.aspect_ratio;
            let [win_w, win_h] = frame.texture_size();
            let win_w = win_w as f32;
            let win_h = win_h as f32;
            f32::min(win_w / w, win_h / h)
        };
        let draw_obj = &draw_obj
            .scale(scale / window.scale_factor())
            .translate((-vec2(model.aspect_ratio, 1.0) / 2.0).extend(0.0));
        draw(app, &model.user_model, draw_obj);
        draw_obj.to_frame(app, &frame).unwrap();
    }
}

fn exit<M>(app: &App, model: Model<M>) {
    if let Some(capture_state) = model.capture_state {
        capture_state
            .frame_capturer
            .wait_for_image_writing(&app.main_window());
    }
    if let Some(exit) = model.events.exit {
        exit(app, model.user_model);
    }
}
