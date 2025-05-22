from flask import Flask, render_template, Response
import cv2
import threading
import os
import logging

class VisionServer:
    def __init__(self):
        # Set up logging
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)
        
        # Get the root directory (where main.py is)
        root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        template_dir = os.path.join(root_dir, 'templates')
        
        self.logger.debug(f"Root directory: {root_dir}")
        self.logger.debug(f"Template directory: {template_dir}")
        
        if not os.path.exists(template_dir):
            self.logger.error(f"Template directory not found: {template_dir}")
            raise FileNotFoundError(f"Template directory not found: {template_dir}")
        
        self.app = Flask(__name__, 
                        template_folder=template_dir)
        self.current_frame = None
        self._setup_routes()

    def _setup_routes(self):
        @self.app.route('/')
        def index():
            try:
                return render_template('index.html')
            except Exception as e:
                self.logger.error(f"Error rendering template: {str(e)}")
                return f"Error: {str(e)}", 500

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')

    def _generate_frames(self):
        while True:
            try:
                if self.current_frame is not None:
                    # Convert frame to JPEG
                    _, buffer = cv2.imencode('.jpg', self.current_frame)
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            except Exception as e:
                self.logger.error(f"Error generating frame: {str(e)}")
            threading.Event().wait(0.033)  # ~30 FPS

    def update_frame(self, frame):
        self.current_frame = frame

    def start(self):
        """Start the Flask server in a separate thread"""
        self.logger.info("Starting Flask server...")
        server_thread = threading.Thread(
            target=lambda: self.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True),
            daemon=True
        )
        server_thread.start()
        self.logger.info("Flask server started")
