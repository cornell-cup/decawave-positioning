const fs = require('fs');
const express = require('express');
const WebSocket = require('ws');

const app = express();
const server = require('http').Server(app);
const wss = new WebSocket.Server({ server });

const SERVER_PORT = 9000;

app.use(express.static('public'));

let tags = [];
let position = "0 0 0";

wss.on('connection', function(ws) {
  console.log('Client connected');
  ws.on('message', function(msg) {
    const data = JSON.parse(msg);
    if (data.type === "get tags") {
      ws.send(JSON.stringify({ "type": "tags", "tags": tags }))
    }
    else if (data.type === "get position") {
      ws.send(JSON.stringify({ "type": "position", "position": position }));
    }
    else if (data.type === "set tags") {
      tags = data.tags;
    }
    else if (data.type === "set position") {
      position = data.position;
    }
  });
});

server.listen(SERVER_PORT);
console.log('Opened server at port', SERVER_PORT);
