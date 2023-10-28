import webbrowser
import threading
import time
from codigo_funcional_GPS import GPSPositionTracker

class MapDisplay:
    def __init__(self):
        self.api_key = "AIzaSyAsPUeFxVnRA2d6zz5Ava5ZCSUtq8KGKhA"
        self.html_template = '/home/raapl/Downloads/python-gps-examples/templates/Index.html'
        self.html_output = 'temp_Index.html'
        self.gps_tracker = GPSPositionTracker()

    def show_location_on_map(self, latitude, longitude):
        with open(self.html_template, 'r') as file:
            content = file.read()
            content = content.replace(self.api_key, self.api_key)
            content = content.replace("{{latitude}}", str(latitude))
            content = content.replace("{{longitude}}", str(longitude))

        with open(self.html_output, 'w') as file:
            file.write(content)

        # Abre o mapa no navegador padrão
        webbrowser.open(self.html_output)
    def display_current_location(self):
    # Starting the GPS tracker in a separate thread
      threading.Thread(target=self.gps_tracker.run).start()
    # Waiting for a few seconds to ensure we have a GPS reading
      time.sleep(5)
    # Obtendo as coordenadas
      latitude = self.gps_tracker.current_latitude
      longitude = self.gps_tracker.current_longitude
    # Se latitude e longitude são válidos, mostra no mapa
      if latitude and longitude:
        # This line opens the Flask server in the default browser
        webbrowser.open('http://0.0.0.0:5000/')

if __name__ == "__main__":
    map_display = MapDisplay()
    map_display.display_current_location()


