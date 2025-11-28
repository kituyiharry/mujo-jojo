use std::fs::File;
use std::io::Write;
use std::path::Path;
use image::{ImageBuffer, Rgb};
use rav1e::prelude::*;

pub struct VideoEncoder {
    config: Config,
    context: Context<u8>,
    frame_count: u64,
}

impl VideoEncoder {
    pub fn new(width: usize, height: usize, fps: u64) -> Result<Self, Box<dyn std::error::Error>> {
        let enconf = EncoderConfig {
            width, 
            height,
            time_base: Rational::new(1, fps),
            bit_depth: 8,
            speed_settings: SpeedSettings::from_preset(9),
            ..Default::default()
        };
        let config = Config::new().with_encoder_config(enconf.clone());
        let context = config.new_context()?;
        Ok(Self {
            config,
            context,
            frame_count: 0,
        })
    }
    
    pub fn add_frame(&mut self, img: ImageBuffer<Rgb<u8>, Vec<u8>>) -> Result<(), Box<dyn std::error::Error>> {
        let (width, height) = img.dimensions();
        let rgb_img = img;
        
        // Create a rav1e frame
        let mut frame = self.context.new_frame();
        
        // Convert RGB to YUV420
        let yuv = rgb_to_yuv420(&rgb_img, width as usize, height as usize);
        
        // Fill the frame planes
        for (plane_idx, plane_data) in yuv.iter().enumerate() {
            let plane = &mut frame.planes[plane_idx];
            for (dst, &src) in plane.data.iter_mut().zip(plane_data.iter()) {
                *dst = src;
            }
        }
        
        self.context.send_frame(frame)?;
        self.frame_count += 1;
        
        Ok(())
    }
    
    pub fn finish(mut self, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        // Flush remaining frames
        self.context.flush();
        
        let mut output_file = File::create(output_path)?;
        
        // Process all packets
        loop {
            match self.context.receive_packet() {
                Ok(packet) => {
                    // Write packet data
                    output_file.write_all(&packet.data)?;
                }
                Err(EncoderStatus::Encoded) => { 
                    continue 
                },
                Err(EncoderStatus::LimitReached) => break,
                Err(e) => return Err(format!("Encoding error: {:?}", e).into()),
            }
        }
        Ok(())
    }
}

// Simple RGB to YUV420 conversion
fn rgb_to_yuv420(rgb: &ImageBuffer<Rgb<u8>, Vec<u8>>, width: usize, height: usize) -> [Vec<u8>; 3] {
    let mut y_plane = vec![0u8; width * height];
    let mut u_plane = vec![0u8; (width / 2) * (height / 2)];
    let mut v_plane = vec![0u8; (width / 2) * (height / 2)];
    
    // Convert RGB to YUV
    for y in 0..height {
        for x in 0..width {
            let pixel = rgb.get_pixel(x as u32, y as u32);
            let r = pixel[0] as f32;
            let g = pixel[1] as f32;
            let b = pixel[2] as f32;
            
            // ITU-R BT.601 conversion
            let y_val = (0.299 * r + 0.587 * g + 0.114 * b) as u8;
            y_plane[y * width + x] = y_val;
            
            // Subsample U and V (4:2:0)
            if x % 2 == 0 && y % 2 == 0 {
                let u_val = ((-0.169 * r - 0.331 * g + 0.500 * b) + 128.0) as u8;
                let v_val = ((0.500 * r - 0.419 * g - 0.081 * b) + 128.0) as u8;
                
                let uv_idx = (y / 2) * (width / 2) + (x / 2);
                u_plane[uv_idx] = u_val;
                v_plane[uv_idx] = v_val;
            }
        }
    }
    
    [y_plane, u_plane, v_plane]
}

