// port
const SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");
// server
const express = require("express");
const app = express();
const http = require("http").Server(app);
const io = require("socket.io")(http);
const open = require("open");

// Defining the serial port
var port = new SerialPort("COM9", {
  baudRate: 9600,
});

// Read the data from the serial port
const parser = port.pipe(new Readline());
parser.on("data", (line) => {
  [lon, lat, dist] = line.split(",");
  lon = angle_to_decimal(lon);
  lat = angle_to_decimal(lat);
  io.emit("new coordinate", { lon, lat, dist });
});

open("http://localhost:3000");

app.use(express.static(__dirname + "/public"));

app.get("/", (req, res) => {
  res.sendFile(__dirname + "/index.html");
});

http.listen(3000);

function angle_to_decimal(nmea) {
  let degrees = parseInt(nmea / 100);
  let minutes = nmea - degrees * 100;
  let dec = degrees + minutes / 60;
  return dec;
}