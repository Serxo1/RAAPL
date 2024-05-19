from flask import Flask, render_template, request, jsonify
from controller.real_oficial import RobotController, GPSController, CompassController

app = Flask(__name__)

gps_controller = GPSController()
compass_controller = CompassController()
robot_controller = RobotController(gps_controller, compass_controller)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/set_waypoints', methods=['POST'])
def set_waypoints():
    data = request.get_json()
    latitude = data['latitude']
    longitude = data['longitude']
    print(f"Received waypoint: Latitude = {latitude}, Longitude = {longitude}")  # Log para debug
    robot_controller.set_destination(latitude, longitude)
    return jsonify({'status': 'success', 'message': 'Waypoint set successfully'})
    
@app.route('/current_location')
def current_location():
    latitude, longitude = gps_controller.get_location()
    if latitude is None or longitude is None:
        # Defina um fallback caso não haja dados GPS disponíveis
        latitude, longitude = -34.397, 150.644  # Localização padrão
    return jsonify({'latitude': latitude, 'longitude': longitude})

@app.route('/add_waypoint', methods=['POST'])
def add_waypoint():
    data = request.get_json()
    robot_controller.add_waypoint(data['latitude'], data['longitude'])
    return jsonify({'status': 'success', 'message': 'Waypoint added'})

@app.route('/start_navigation', methods=['POST'])
def start_navigation():
    robot_controller.start_navigation()
    return jsonify({'status': 'success', 'message': 'Navigation started'})
    
@app.route('/cancel_navigation', methods=['POST'])
def cancel_navigation():
    robot_controller.stop()
    return jsonify({'status': 'success', 'message': 'Navigation cancelled'})

@app.route('/clear_waypoints', methods=['POST'])
def clear_waypoints():
    robot_controller.clear_waypoints()  # Supondo que exista uma função para limpar waypoints
    return jsonify({'status': 'success', 'message': 'Waypoints cleared'})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
