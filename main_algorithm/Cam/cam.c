#include <libcamera/libcamera.h>
#include <libcamera/framebuffer.h>
#include <libcamera/buffer.h>
#include <libcamera/stream.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <cstring>

using namespace libcamera;

int main(int argc, char *argv[]) {
    // Inicializa el gestor de cámara
    CameraManager cameraManager;
    if (cameraManager.start() != 0) {
        std::cerr << "Error al iniciar el gestor de la cámara." << std::endl;
        return -1;
    }

    // Obtén la primera cámara disponible
    if (cameraManager.cameras().empty()) {
        std::cerr << "No se encontraron cámaras conectadas." << std::endl;
        cameraManager.stop();
        return -1;
    }

    std::shared_ptr<Camera> camera = cameraManager.cameras()[0];
    if (!camera) {
        std::cerr << "Error al obtener la cámara." << std::endl;
        cameraManager.stop();
        return -1;
    }

    // Adquiere la cámara
    if (camera->acquire() != 0) {
        std::cerr << "No se pudo adquirir la cámara." << std::endl;
        cameraManager.stop();
        return -1;
    }

    // Configura la cámara
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::StillCapture });
    if (!config) {
        std::cerr << "Error al generar la configuración de la cámara." << std::endl;
        camera->release();
        cameraManager.stop();
        return -1;
    }

    // Ajusta la configuración
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.pixelFormat = formats::RGB888;
    streamConfig.size.width = 1280;
    streamConfig.size.height = 720;

    if (camera->configure(config.get()) != 0) {
        std::cerr << "Error al configurar la cámara." << std::endl;
        camera->release();
        cameraManager.stop();
        return -1;
    }

    // Asigna búferes
    FrameBufferAllocator allocator(camera);
    if (allocator.allocate(streamConfig.stream()) < 0) {
        std::cerr << "Error al asignar los búferes." << std::endl;
        camera->release();
        cameraManager.stop();
        return -1;
    }

    std::vector<std::unique_ptr<FrameBuffer>> buffers;
    for (const auto &buffer : allocator.buffers(streamConfig.stream())) {
        buffers.push_back(std::move(buffer));
    }

    // Comienza la cámara
    if (camera->start() != 0) {
        std::cerr << "Error al iniciar la cámara." << std::endl;
        camera->release();
        cameraManager.stop();
        return -1;
    }

    // Coloca el primer búfer en la cola
    Request *request = camera->createRequest();
    if (!request) {
        std::cerr << "Error al crear la solicitud de captura." << std::endl;
        camera->stop();
        camera->release();
        cameraManager.stop();
        return -1;
    }

    for (auto &buffer : buffers) {
        if (request->addBuffer(streamConfig.stream(), buffer.get()) != 0) {
            std::cerr << "Error al agregar un búfer a la solicitud." << std::endl;
            delete request;
            camera->stop();
            camera->release();
            cameraManager.stop();
            return -1;
        }
    }

    // Realiza la captura
    if (camera->queueRequest(request) != 0) {
        std::cerr << "Error al colocar la solicitud en la cola." << std::endl;
        delete request;
        camera->stop();
        camera->release();
        cameraManager.stop();
        return -1;
    }

    // Espera a que se complete la captura
    camera->waitForRequestCompleted(request);

    // Guarda la imagen en un archivo
    FrameBuffer *capturedBuffer = request->buffers().begin()->second;
    if (!capturedBuffer) {
        std::cerr << "Error al capturar el búfer." << std::endl;
        delete request;
        camera->stop();
        camera->release();
        cameraManager.stop();
        return -1;
    }

    const FrameMetadata &metadata = capturedBuffer->metadata();
    if (metadata.status != FrameMetadata::FrameStatus::Complete) {
        std::cerr << "La captura de la imagen no se completó correctamente." << std::endl;
        delete request;
        camera->stop();
        camera->release();
        cameraManager.stop();
        return -1;
    }

    // Escribir los datos de la imagen en un archivo
    const uint8_t *data = static_cast<const uint8_t *>(capturedBuffer->planes()[0].data());
    std::ofstream outFile("captura.raw", std::ios::binary);
    outFile.write(reinterpret_cast<const char *>(data), capturedBuffer->planes()[0].length);
    outFile.close();

    std::cout << "Imagen capturada y guardada como 'captura.raw'." << std::endl;

    // Limpieza
    delete request;
    camera->stop();
    camera->release();
    cameraManager.stop();

    return 0;
}
