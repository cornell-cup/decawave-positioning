const COM_PORT = 'COM16';
const COM_BAUDRATE = 115200;
const SERVER_PORT = 9000;

const fs = require('fs');
const express = require('express');
const app = express();
const server = require('http').Server(app);
const io = require('socket.io')(server);
const SerialPort = require("serialport");

const port = new SerialPort(COM_PORT, {
  baudRate: COM_BAUDRATE,
  parser: SerialPort.parsers.readline('\n')
});

port.on('open', function() {
  console.log('Serial: Opened serial connection at port', COM_PORT);
});
port.on('error', function(err) {
  console.log('Serial Error: ', err.message);
});

// Total number of tags
const NUM_TAGS = 3;
// Array of tag info
// tag[i].distance is the current distance from the anchor to tag[i]
// tag[i].to[j] is the distance from tag[i] to tag[j]
// Because of sampling, tag[i].to[j] might not equal tag[j].to[i]
let tags = new Array(NUM_TAGS + 1);
for (let i = 0; i <= NUM_TAGS; i++) {
  tags[i] = { distance: 0.0, to: new Array(NUM_TAGS + 1) };
  for (let j = 0; j <= NUM_TAGS; j++) {
    tags[i].to[j] = -1.0;
  }
}
// DEBUG
tags = JSON.parse(fs.readFileSync("tags.json"));

port.on('data', function(data) {
  // Get a fragment of data
  const str = data.toString('utf8').trim();
  try {
    // Parse the data into a tag and distance
    const parts = str.split(",");
    if (parts[0] == "P") { // Poll result
      const tag = parseInt(parts[1]);
      const distance = parseFloat(parts[2]);
      if (tag >= 0 && tag <= NUM_TAGS) {
        // TODO filter the distance, or filter the resulting position
        tags[tag].distance = distance;
        console.log('Serial: Read tag', tag, 'distance', distance);
      }
    }
    else if (parts[0] == "Q") { // Query result
      const tagFrom = parseInt(parts[1]);
      const tagTo = parseInt(parts[2]);
      const samples = parseInt(parts[3]);
      const distanceTotal = parseFloat(parts[4]);
      if (samples > 0 && tagFrom >= 0 && tagFrom <= NUM_TAGS && tagTo >= 0 && tagTo <= NUM_TAGS) {
        const distance = distanceTotal / samples;
        tags[tagFrom].to[tagTo] = distance;
        console.log('Serial: Read query from', tagFrom, 'to', tagTo, 'distance', distance);
      }
    }
  }
  catch (e) {
  }
});

process.stdin.setRawMode(true);
process.stdin.on('readable', function () {
  const chunk = process.stdin.read();
  if (chunk == null) {
    return;
  }
  const key = String.fromCharCode(chunk[0]);
  if (port.isOpen()) {
    if (key == 'p') { // Write polling mode
      port.write('P');
      console.log("Serial: Switching to poll mode");
    }
    else if (key == 'q') { // Write query mode
      port.write('Q');
      console.log("Serial: Switching to query mode");
    }
    else if (chunk[0] == 27) {
      process.exit();
    }
  }
});

server.listen(SERVER_PORT);
console.log('Opened server at port', SERVER_PORT);

app.use(express.static('public'));

io.on('connection', function(socket) {
  console.log('Client connected');
  socket.on('request distances', function(data) {
    socket.emit('distances', tags.map(tag => tag.distance));
  });
  socket.on('request tags', function(data) {
    socket.emit('tags', tags);
  });
});
