from flask import Flask, Response, render_template_string
import threading
import cv2
import time

class WebServer:
    def __init__(self,vision):
    self.app = Flask(__name__)
    self.vision = vision

        @self.app.route('/')
        def index():
            return render_template_string("""
            <html>
            <head><title>Robot Camera Feed</title><head>
            <body>
                <h1>Live Feed</h1>
                <img src = "{{ url_for('video_feed') }} width="640" height"480">
            </body>
            </html>
            """)
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def gen(self):
        while True:
            frame = self.vision.get_frame()
            if frame is None:
                continue
            
  