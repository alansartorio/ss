use std::path::PathBuf;

use nannou::prelude::*;

pub enum CaptureMode {
    NoCapture,
    Capture { directory: PathBuf },
}

pub struct FrameCapturer {
    // The texture that we will draw to.
    texture: wgpu::Texture,
    // Create a `Draw` instance for drawing to our texture.
    draw: nannou::Draw,
    // The type used to render the `Draw` vertices to our texture.
    renderer: nannou::draw::Renderer,
    // The type used to capture the texture.
    texture_capturer: wgpu::TextureCapturer,
    // The type used to resize our texture to the window texture.
    pub texture_reshaper: wgpu::TextureReshaper,

    capture_mode: CaptureMode,

    frame_count: usize,
}

impl FrameCapturer {
    pub fn new(window: &Window, texture_size: [u32; 2], capture_mode: CaptureMode) -> Self {
        // Retrieve the wgpu device.
        let device = window.device();

        // Create our custom texture.
        let sample_count = window.msaa_samples();
        let texture = wgpu::TextureBuilder::new()
            .size(texture_size)
            // Our texture will be used as the RENDER_ATTACHMENT for our `Draw` render pass.
            // It will also be SAMPLED by the `TextureCapturer` and `TextureResizer`.
            .usage(wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING)
            // Use nannou's default multisampling sample count.
            .sample_count(sample_count)
            // Use a spacious 16-bit linear sRGBA format suitable for high quality drawing.
            .format(wgpu::TextureFormat::Rgba16Float)
            // Build it!
            .build(device);

        // Create our `Draw` instance and a renderer for it.
        let draw = nannou::Draw::new();
        let descriptor = texture.descriptor();
        let renderer =
            nannou::draw::RendererBuilder::new().build_from_texture_descriptor(device, descriptor);

        // Create the texture capturer.
        let texture_capturer = wgpu::TextureCapturer::default();

        // Create the texture reshaper.
        let texture_view = texture.view().build();
        let texture_sample_type = texture.sample_type();
        let texture_sample_type = wgpu::TextureSampleType::Float { filterable: false };
        let dst_format = nannou::Frame::TEXTURE_FORMAT;
        let texture_reshaper = wgpu::TextureReshaper::new(
            device,
            &texture_view,
            sample_count,
            texture_sample_type,
            1,
            dst_format,
        );

        if let CaptureMode::Capture { directory } = &capture_mode {
            // Make sure the directory where we will save images to exists.
            std::fs::create_dir_all(directory).unwrap();
        }

        Self {
            texture,
            draw,
            renderer,
            texture_capturer,
            texture_reshaper,
            capture_mode,
            frame_count: 0,
        }
    }

    pub fn render_to_texture(&mut self, window: &Window) {
        // Render our drawing to the texture.
        let device = window.device();
        let ce_desc = wgpu::CommandEncoderDescriptor {
            label: Some("texture renderer"),
        };
        let mut encoder = device.create_command_encoder(&ce_desc);
        self.renderer
            .render_to_texture(device, &mut encoder, &self.draw, &self.texture);

        match &self.capture_mode {
            CaptureMode::NoCapture => {
                // Submit the commands for our drawing and texture capture to the GPU.
                window.queue().submit(Some(encoder.finish()));
            }
            CaptureMode::Capture { directory } => {
                // Take a snapshot of the texture. The capturer will do the following:
                //
                // 1. Resolve the texture to a non-multisampled texture if necessary.
                // 2. Convert the format to non-linear 8-bit sRGBA ready for image storage.
                // 3. Copy the result to a buffer ready to be mapped for reading.
                let snapshot = self
                    .texture_capturer
                    .capture(device, &mut encoder, &self.texture);

                // Submit the commands for our drawing and texture capture to the GPU.
                window.queue().submit(Some(encoder.finish()));

                // Submit a function for writing our snapshot to a PNG.
                //
                // NOTE: It is essential that the commands for capturing the snapshot are `submit`ted before we
                // attempt to read the snapshot - otherwise we will read a blank texture!
                let path = directory
                    .join(format!("{:05}", self.frame_count))
                    .with_extension("png");
                snapshot.read(move |result| {
                    let image = result.expect("failed to map texture memory").to_owned();
                    image
                        .save(&path)
                        .expect("failed to save texture to png image");
                });
            }
        }
        self.frame_count += 1;
    }

    pub fn get_draw(&self) -> &Draw {
        &self.draw
    }

    pub fn draw_to_texture(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        texture: &wgpu::TextureViewHandle,
    ) {
        self.texture_reshaper.encode_render_pass(texture, encoder);
    }

    pub fn draw_to_frame(&self, frame: Frame) {
        // Sample the texture and write it to the frame.
        let mut encoder = frame.command_encoder();
        self.texture_reshaper
            .encode_render_pass(frame.texture_view(), &mut encoder);
    }

    pub fn wait_for_image_writing(&self, window: &Window) {
        let device = window.device();
        self.texture_capturer
            .await_active_snapshots(device)
            .unwrap();
    }
}
