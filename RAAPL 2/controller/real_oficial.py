import threading
import time
import math
import RPi.GPIO as GPIO
from gpsdclient import GPSDClient
from smbus2 import SMBus
from math import radians, cos, sin, atan2, degrees

class CompassController:
    def __init__(self):
        self.bus = SMBus(1)
        self.address = 0x1E
        self.direction = None
        self.running = True
        self.setup_compass()
        self.thread = threading.Thread(target=self.update_direction)
        self.thread.start()

    def setup_compass(self):
        try:
            # Configura o sensor de bússola
            self.bus.write_byte_data(self.address, 0x00, 0x70)  # Configuração do registro A
            self.bus.write_byte_data(self.address, 0x01, 0xA0)  # Configuração do registro B
            self.bus.write_byte_data(self.address, 0x02, 0x00)  # Modo contínuo
            print("Sensor de bússola configurado")
        except Exception as e:
            print("Erro ao configurar o sensor de bússola:", e)

    def update_direction(self):
        while self.running:
            try:
                # Adiciona log para verificar se a função está sendo chamada
                print("Atualizando direção...")

                # Lê dados da bússola
                data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
                print(f"Dados brutos da bússola: {data}")

                # Processa os dados
                x = (data[0] << 8) | data[1]
                y = (data[4] << 8) | data[5]

                # Compensa para valores negativos
                if x >= 0x8000:
                    x = -((65535 - x) + 1)
                if y >= 0x8000:
                    y = -((65535 - y) + 1)

                print(f"Valores compensados: x={x}, y={y}")

                self.direction = 180 * math.atan2(y, x) / math.pi
                print(f"Nova direção: {self.direction}")

            except Exception as e:
                print("Erro ao ler dados da bússola:", e)
            time.sleep(0.5)

    def get_direction(self):
        return self.direction

    def stop(self):
        self.running = False
        self.thread.join()

class GPSController:
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.client = GPSDClient(host="localhost")
        self.running = True
        self.thread = threading.Thread(target=self.update_location)
        self.thread.start()

    def update_location(self):
        while self.running:
            try:
                for report in self.client.dict_stream(convert_datetime=True):
                    if "lat" in report and "lon" in report:
                        self.latitude = report['lat']
                        self.longitude = report['lon']
                        print(f"Updated GPS: Latitude: {self.latitude}, Longitude: {self.longitude}")
                    time.sleep(0.5)  # Control the update rate
            except Exception as e:
                print("Erro ao ler dados do GPS:", e)

    def get_location(self):
        return self.latitude, self.longitude

    def stop(self):
        self.running = False
        self.thread.join()

class RobotController:
    def __init__(self, gps_controller, compass_controller):
        self.gps_controller = gps_controller
        self.compass_controller = compass_controller
        self.stop_navigation = threading.Event()
        self.lock = Lock()
        self.waypoints = []  # Lista para armazenar múltiplos waypoints
        self.current_target_index = 0  # Índice do waypoint atual
        self.movement_thread = threading.Thread(target=self.navigate)
        self.orientation_threshold = 5  # graus
        self.setup_motors()

    def setup_motors(self):
        # Configurações dos motores (assumindo que já estão corretas)
        self.leftForward = 11
        self.leftBackward = 15
        self.leftEnable = 33
        self.rightForward = 16
        self.rightBackward = 18
        self.rightEnable = 32
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.leftForward, GPIO.OUT)
        GPIO.setup(self.leftBackward, GPIO.OUT)
        GPIO.setup(self.leftEnable, GPIO.OUT)
        GPIO.setup(self.rightForward, GPIO.OUT)
        GPIO.setup(self.rightBackward, GPIO.OUT)
        GPIO.setup(self.rightEnable, GPIO.OUT)
        self.leftEnPWM = GPIO.PWM(self.leftEnable, 100)
        self.rightEnPWM = GPIO.PWM(self.rightEnable, 100)
        self.leftEnPWM.start(0)
        self.rightEnPWM.start(0)

    def add_waypoint(self, latitude, longitude):
        with self.lock:
            self.waypoints.append((latitude, longitude))
            print(f"Waypoint added: Latitude {latitude}, Longitude {longitude}")

    def start_navigation(self):
        with self.lock:
            if not self.movement_thread.is_alive():
                self.stop_navigation.clear()  # Garante que o sinal de parada esteja limpo
                self.movement_thread = threading.Thread(target=self.navigate)
                self.movement_thread.start()
                print("Navigation started")
            else:
                print("Navigation is already active")

    def navigate(self):
        print("Starting navigation...")
        while not self.stop_navigation.is_set():
            if self.current_target_index >= len(self.waypoints):
                print("All waypoints completed")
                break

            with self.lock:
                target_latitude, target_longitude = self.waypoints[self.current_target_index]

            current_latitude, current_longitude = self.gps_controller.get_location()
            if current_latitude is None or current_longitude is None:
                print("Error: Could not obtain current robot location.")
                continue

            if self.is_close_enough(current_latitude, current_longitude, target_latitude, target_longitude):
                print("Destination reached!")
                self.stop_motors()
                with self.lock:
                    self.current_target_index += 1  # Move to next waypoint
                if self.current_target_index >= len(self.waypoints):
                    continue

            target_direction = self.calculate_target_direction(current_latitude, current_longitude, target_latitude, target_longitude)
            current_direction = self.compass_controller.get_direction()

            self.adjust_robot_orientation(current_direction, target_direction)
            self.move_forward()
            time.sleep(0.5)  # Breve pausa para não sobrecarregar o loop

        self.stop_motors()
        print("Navigation stopped.")

    def is_close_enough(self, current_latitude, current_longitude, target_latitude, target_longitude, proximity_threshold=0.0001):
        return (abs(current_latitude - target_latitude) < proximity_threshold and 
                abs(current_longitude - target_longitude) < proximity_threshold)

    def calculate_target_direction(self, current_latitude, current_longitude, target_latitude, target_longitude):
        # Converte graus em radianos
        from math import radians, cos, sin, atan2

        # Conversão das coordenadas de graus para radianos
        lat1, lon1 = radians(current_latitude), radians(current_longitude)
        lat2, lon2 = radians(target_latitude), radians(target_longitude)

        # Cálculo das diferenças de coordenadas
        delta_lon = lon2 - lon1

        # Cálculo do ângulo de direção em radianos
        x = cos(lat2) * sin(delta_lon)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
        bearing = atan2(x, y)

        # Conversão de radianos para graus e normalização do ângulo
        bearing = (degrees(bearing) + 360) % 360

        return bearing
    
    def adjust_robot_orientation(self, current_direction, target_direction):
        angle_difference = self.calculate_angle_difference(current_direction, target_direction)
        while abs(angle_difference) > self.orientation_threshold:
            if angle_difference > 0:
                self.turn_right()
            else:
                self.turn_left()
            
            time.sleep(0.1)  # Brief pause for adjustment
            current_direction = self.compass_controller.get_direction()
            angle_difference = self.calculate_angle_difference(current_direction, target_direction)
            
            print(f"Adjusting orientation: current_direction={current_direction}, target_direction={target_direction}, angle_difference={angle_difference}")
            
            # Adiciona uma condição de saída de segurança para evitar loop infinito
            if self.stop_navigation.is_set():
                self.stop_motors()
                break
            
        print("Orientation adjusted.")

    def calculate_angle_difference(self, current_direction, target_direction):
        angle_difference = target_direction - current_direction
        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360
        return angle_difference

    def move_forward(self):
        if self.stop_navigation.is_set():
            print("Stop signal received, stopping...")
            self.stop_motors()
            return
        print("Se movendo para frente")
        GPIO.output(self.leftForward, GPIO.HIGH)
        GPIO.output(self.rightForward, GPIO.HIGH)
        self.leftEnPWM.ChangeDutyCycle(80)
        self.rightEnPWM.ChangeDutyCycle(80)

    def turn_right(self):
        if self.stop_navigation.is_set():
            print("Stop signal received, stopping...")
            self.stop_motors()
            return
            
        print("Se movendo para direita")
        GPIO.output(self.leftForward, GPIO.HIGH)
        GPIO.output(self.rightBackward, GPIO.HIGH)
        self.leftEnPWM.ChangeDutyCycle(80)
        self.rightEnPWM.ChangeDutyCycle(80)

    def turn_left(self):
        if self.stop_navigation.is_set():
            print("Stop signal received, stopping...")
            self.stop_motors()
            return
        print("Se movendo para esquerda")
        GPIO.output(self.leftBackward, GPIO.HIGH)
        GPIO.output(self.rightForward, GPIO.HIGH)
        self.leftEnPWM.ChangeDutyCycle(80)
        self.rightEnPWM.ChangeDutyCycle(80)

    def stop_motors(self):
        if self.stop_navigation.is_set():
            print("Stop signal received, stopping...")
            self.stop_motors()
            return
        print("motores parando")
        GPIO.output(self.leftForward, GPIO.LOW)
        GPIO.output(self.leftBackward, GPIO.LOW)
        GPIO.output(self.rightForward, GPIO.LOW)
        GPIO.output(self.rightBackward, GPIO.LOW)
        self.leftEnPWM.ChangeDutyCycle(0)
        self.rightEnPWM.ChangeDutyCycle(0)
    
    def stop(self):
        print("Stopping navigation...")
        self.stop_navigation.set()  # Sinaliza para a thread parar
        if self.movement_thread.is_alive():
            self.movement_thread.join()  # Espera a thread terminar
        self.stop_motors()  # Garante que os motores estão parados
        print("Navigation stopped.")
