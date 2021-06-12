// this file is just for testing the server without connection to the port

const express = require("express");
const app = express();
const http = require("http").Server(app);
const io = require("socket.io")(http);
const open = require("open");

open("http://localhost:3000");

app.use(express.static(__dirname + "/public"));


let coordinates = [
  "30.12177, 31.34205, 0",
  "30.12185, 31.34215, 10",
  "30.1219, 31.34222, 20",
  "30.12196, 31.34229, 30",
  "30.122, 31.34233, 40",
  "30.12206, 31.34241, 50",
  "30.12212, 31.34247, 60",
  "30.12219, 31.34256, 70",
  "30.12226, 31.34264, 80",
  "30.12226, 31.34264, 90",
  "30.12229, 31.3427, 100",
  "30.12226, 31.34277, 110",
  "30.12223, 31.34285, 120",
  "30.12219, 31.34296, 130",
  "30.12215, 31.34309, 140",
  "30.12214, 31.34313, 150",
  "30.12209, 31.34329, 160",
  "30.12204, 31.34339, 170",
  "30.12199, 31.3433, 180",
  "30.12192, 31.34318, 190",
];

  let interval = setInterval(() => {
    if (coordinates.length) {
      [lon, lat, dist] = coordinates[0].split(",");
      io.emit("new coordinate", { lon, lat, dist });
      coordinates.shift();
      if (!coordinates.length) clearInterval(interval);
    }
  }, 3000);

  app.get("/", (req, res) => {
    res.sendFile(__dirname + "/index.html");
  });

http.listen(3000);