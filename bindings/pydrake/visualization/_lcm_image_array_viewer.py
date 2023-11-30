import argparse
import datetime
import time

from flask import Flask, Response

from pydrake.lcm import DrakeLcm
from pydrake.visualization import (
    _LcmImageStreamWorker,
)


class _Server(Flask):
    """Streams LCM camera images via the HTTP protocol."""

    def __init__(self, *, channel_regex):
        super().__init__("lcm_image_array_viewer")
        channel_regex = "DRAKE_RGBD_CAMERA_IMAGES.*"
        self._pnger = _LcmImageStreamWorker(lcm_url="",
                                            channel_regex=channel_regex)
        self.add_url_rule("/", view_func=self._index)
        self.add_url_rule("/channel/<name>", view_func=self._channel)

    def _index(self):
        mime_type, data = self._make_index()
        return Response(data, mimetype=mime_type)

    def _make_index(self):
        mime_type = "text/html"
        page = f"""
<html>
<head>
<meta http-equiv="refresh" content="1">
</head>
<body>
<h1>LCM Image Viewer</h1>
As of {datetime.datetime.now().ctime()} the available channels are:
<ul>
"""
        channel_names = self._pnger.GetChannelNames()
        if not channel_names:
            page += "(none)"
        else:
            for name in channel_names:
                page += f'<li><a href="/channel/{name}" target="_blank">{name}</a>\n'  # noqa
        page += """\
</ul>
</body>
</html>"""
        return (mime_type, page.encode("utf-8"))

    def _channel(self, *, name):
        return self._respond(content_generator=self._channel_generator(name))

    def _respond(self, *, content_generator):
        return Response(
            self._response_generator(content_generator),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def _response_generator(self, content_generator):
        for mime_type, data in content_generator:
            yield (
                b"--frame\r\nContent-Type: "
                + mime_type.encode("utf-8")
                + b"\r\n\r\n"
                + data
                + b"\r\n"
            )

    def _channel_generator(self, name):
        mime_type = "image/png"
        while True:
            png_bytes = self._pnger.GetLatestPng(name)
            if png_bytes is not None:
                yield (mime_type, png_bytes)
            else:
                time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="URL to host on, default: 127.0.0.1.",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=False,
        default=8000,
        help="Port to host on, default: 8000.",
    )
    parser.add_argument(
        "--channel_regex",
        type=str,
        default="DRAKE_RGBD_CAMERA_IMAGES.*",
        help="The LCM channel(s) to subscribe to.",
    )
    args = parser.parse_args()

    # Instantiate a `_Server` and run it.
    server = _Server(channel_regex=args.channel_regex)
    server.run(host=args.host, port=args.port, debug=True)


if __name__ == "__main__":
    main()
