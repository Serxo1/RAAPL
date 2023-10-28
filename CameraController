from flask import Flask, render_template, Response
import os
import tempfile

class VideoStreamingApp:
    def __init__(self):
        self.app = Flask(__name__)
        self.setup_routes()

    def setup_routes(self):
        @self.app.route('/')
        def index():
            """Video streaming home page."""
            return render_template('Index.html')

        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route. Put this in the src attribute of an img tag."""
            return Response(self.gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def gen(self):
        """Video streaming generator function."""
        while True:
            # Use a temporary file to store the captured image
            with tempfile.NamedTemporaryFile(suffix=".jpg") as tempf:
                os.system(f"fswebcam -q {tempf.name}")

                with open(tempf.name, "rb") as f:
                    frame = f.read()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def run(self):
        self.app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)

