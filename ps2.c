#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>

//int serial_port; // Variable global para el puerto serie

// Función para gestionar la entrada de la terminal
void *gestionar_entrada(int *serial_port) {
    char input[2];
    while (1) {
        // Lee la entrada desde la terminal
        if (fgets(input, sizeof(input), stdin) != NULL) {
            input[strcspn(input, "\n")] = '\0'; // Eliminar salto de línea

            // Write to the serial port
            if (write(*serial_port, input, strlen(input)) < 0) {
                perror("Error al escribir en el puerto serie");
            }
        }
    }
}

// Función para leer del puerto serie y mostrar la distancia
void *gestionar_puerto_serie(int *serial_port) {
    char buffer[256];
    while (1) {
        memset(buffer, 0, sizeof(buffer)); // Limpiar el buffer
        int num_bytes = read(*serial_port, buffer, sizeof(buffer) - 1); // Leer datos del puerto serie
        if (num_bytes < 0) {
            perror("Error al leer del puerto serie");
        } else if (num_bytes > 0) {
            buffer[num_bytes] = '\0'; // Asegurar que el buffer sea una cadena válida
            printf("%s", buffer); // Mostrar la distancia
            fflush(stdout); // Asegurar la salida inmediata
        }
    }
}

int main() {
    // Abrir el puerto serie
    int puerto_serie = open("/dev/ttyACM0", O_RDWR); // Cambiar a tu puerto serie correspondiente
    if (puerto_serie < 0) {
        perror("No se pudo abrir el puerto serie");
        return 1;
    }

    // Configurar los atributos del puerto serie
    struct termios tty;
    if (tcgetattr(puerto_serie, &tty) != 0) {
        perror("No se pudieron obtener los atributos del puerto serie");
        close(puerto_serie);
        return 1;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // Configurar 8 bits de datos
    tty.c_iflag &= ~IGNBRK; // Deshabilitar el procesamiento de interrupciones por break
    tty.c_lflag = 0; // Deshabilitar el modo canónico y el eco
    tty.c_oflag = 0; // Deshabilitar mapeos de salida

    tty.c_cc[VMIN] = 1; // Leer al menos 1 byte
    tty.c_cc[VTIME] = 1; // Tiempo de espera de 0.1 segundos

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Deshabilitar control de flujo por software
    tty.c_cflag |= (CLOCAL | CREAD); // Ignorar controles del módem, habilitar lectura
    tty.c_cflag &= ~(PARENB | PARODD); // Deshabilitar la paridad
    tty.c_cflag &= ~CSTOPB; // 1 bit de parada
    tty.c_cflag &= ~CRTSCTS; // Deshabilitar control de flujo por hardware

    if (tcsetattr(puerto_serie, TCSANOW, &tty) != 0) {
        perror("No se pudieron establecer los atributos del puerto serie");
        close(puerto_serie);
        return 1;
    }

    // Crear hilos
    pthread_t hilo_entrada;
    pthread_t hilo_puerto_serie;

    if (pthread_create(&hilo_entrada, NULL, (void*)gestionar_entrada, (void*)&puerto_serie) != 0) {
        perror("Error al crear el hilo de entrada");
        close(puerto_serie);
        return 1;
    }

    if (pthread_create(&hilo_puerto_serie, NULL, (void*)gestionar_puerto_serie, (void*)&puerto_serie) != 0) {
        perror("Error al crear el hilo del puerto serie");
        close(puerto_serie);
        return 1;
    }

    // Esperar a que los hilos terminen
    pthread_join(hilo_puerto_serie, NULL);
    pthread_join(hilo_entrada, NULL);

    // Cerrar el puerto serie
    close(puerto_serie);
    return 0;
}
