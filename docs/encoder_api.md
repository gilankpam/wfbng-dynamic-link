The API for majestic and waybeam is quite similar

Bit rate change (majestic and waybeam):

GET http://localhost/api/v1/set?video0.bitrate={bitrate}


RoiQP (majestic and waybeam):

GET http://localhost/api/v1/set?fpv.roiQp={roiQp}

Video FPS (majestic and waybeam):

GET http://localhost/api/v1/set?video0.fps=90

You can use it together in single request

GET http://localhost/api/v1/set?video0.bitrate={bitrate}&fpv.roiQp={roiQp}&video0.fps=90


Request IDR Frame (majestic and waybeam):

GET http://localhost/request/idr
