import RPi.GPIO as GPIO
import tkinter as tk
from GPSTESTE import MapDisplay
from camera import VideoStreamingApp
from tkinter import font as tkfont
import threading
import webbrowser
from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

# GPIO setup
leftForward = 11
leftBackward = 15
leftEnable = 33

rightForward = 16
rightBackward = 18
rightEnable = 32

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(leftBackward, GPIO.OUT)
GPIO.setup(leftEnable, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)
GPIO.setup(rightBackward, GPIO.OUT)
GPIO.setup(rightEnable, GPIO.OUT)

leftEnPWM = GPIO.PWM(leftEnable, 100)
leftEnPWM.start(50)
GPIO.output(leftEnable, GPIO.HIGH)

rightEnPWM = GPIO.PWM(rightEnable, 100)
rightEnPWM.start(50)
GPIO.output(rightEnable, GPIO.HIGH)


# Functions for robot behaviors with added print statements
def move_forward():
    stop_motors()  # Stop the motors before moving forward
    print("Moving forward...")
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)

def move_backward():
    stop_motors()  # Stop the motors before moving backward
    print("Moving backward...")
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)

def turn_left():
    stop_motors()  # Stop the motors before turning left
    print("Turning left...")
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.HIGH)

def turn_right():
    stop_motors()  # Stop the motors before turning right
    print("Turning right...")
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.LOW)
        
@app.route('/command', methods=['POST'])
def handle_command():
    data = request.get_json()
    command = data.get("command")

    if command == "move_forward":
        move_forward()
        return jsonify({"message": "Moving forward!"})
    elif command == "move_backward":
        move_backward()
        return jsonify({"message": "Moving backward!"})
    elif command == "turn_left":
         turn_left()
         return jsonify({"message": "Turning left!"})
    elif command == "turn_right":
         turn_right()
         return jsonify({"message": "Turning right!"})
    else:
        return jsonify({"message": "Invalid command!"})

def stop_motors():
    print("Stopping motors...")
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.LOW)
        

class Joystick:
    def __init__(self, master):
        # Configuração da aparência da janela principal
        master.configure(bg='#2A2A2A')  # Fundo escuro
        
        # Estilização
        btn_font = tkfont.Font(family="Helvetica", size=14, weight="bold")
        
        self.canvas = tk.Canvas(master, width=200, height=200, bg="#2A2A2A", highlightthickness=0)
        self.canvas.pack(pady=20)
        

        # Desenha a base e o manípulo do joystick com estilização aprimorada
        self.base = self.canvas.create_oval(25, 25, 175, 175, fill="#555555")
        self.handle = self.canvas.create_oval(75, 75, 125, 125, fill="#999999")

        # Crie uma instância da classe VideoStreamingApp
        self.video_stream = VideoStreamingApp()
        
        # Adicionando o botão para exibir a localização atual com estilização aprimorada
        self.map_display = MapDisplay()
        self.locate_button = tk.Button(master, text="Localize meu robô", 
        command=self.start_video_stream, bg="#4CAF50", fg="white", font=btn_font, borderwidth=0, padx=10, pady=5)
        self.locate_button.pack(pady=20)

        
        # Associa eventos de tecla para simular movimento do joystick
        master.bind("w", self.move_up)
        master.bind("s", self.move_down)
        master.bind("a", self.move_left)
        master.bind("d", self.move_right)
        master.bind("<KeyRelease>", self.center_joystick)
        
        

    
    def move_up(self, event):
        self.canvas.coords(self.handle, 75, 25, 125, 75)
        move_forward()

    def move_down(self, event):
        self.canvas.coords(self.handle, 75, 125, 125, 175)
        move_backward()

    def move_left(self, event):
        self.canvas.coords(self.handle, 25, 75, 75, 125)
        turn_left()

    def move_right(self, event):
        self.canvas.coords(self.handle, 125, 75, 175, 125)
        turn_right()

    def run_flask_app(video_stream):
        video_stream.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    
    def start_video_stream(self):
        self.map_display.display_current_location()
        # Start the Flask server in a separate thread
        thread = threading.Thread(target=Joystick.run_flask_app, args=(self.video_stream,))
        thread.daemon = True
        thread.start()
        # Open the Flask server URL in the default web browser
        



    def center_joystick(self, event):
        self.canvas.coords(self.handle, 75, 75, 125, 125)
        stop_motors()

    
root = tk.Tk()
root.title("Joystick Robot Controller")

joystick = Joystick(root)

root.mainloop()
