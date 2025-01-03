#include <librealsense2/rs.hpp> // Incluye la API de RealSense
#include <opencv2/opencv.hpp>   // Incluye OpenCV
#include <iostream>

int main() {
    // Configuración de la cámara
    rs2::pipeline pipe;
    rs2::config cfg;

    // Habilita el stream de color (RGB)
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // Inicia el pipeline con la configuración
    pipe.start(cfg);

    while (true) {
        // Espera a que lleguen frames (color y profundidad, si se usa)
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame(); // Obtenemos el frame de color

        // Convierte el frame de color en un formato que OpenCV puede manejar
        const int width = color_frame.as<rs2::video_frame>().get_width();
        const int height = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat color_image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Muestra la imagen de color en una ventana
        cv::imshow("RGB", color_image);

        // Salir si se presiona la tecla 'q'
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Detenemos el pipeline
    pipe.stop();

    return 0;
}
