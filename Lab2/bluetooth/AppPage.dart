import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:mjpeg_stream/mjpeg_stream.dart';

class AppPage extends StatefulWidget {
  final BluetoothDevice server;

  const AppPage({Key? key, required this.server}) : super(key: key);

  @override
  _AppPage createState() => _AppPage();
}

class _AppPage extends State<AppPage> {
  BluetoothConnection? connection;

  String _messageBuffer = '';
  String message = "";

  bool isConnecting = true;
  bool get isConnected => connection != null && connection!.isConnected;

  bool isDisconnecting = false;

  Future<void> _initBluetooth() async {
    bool hasPermission = await requestPermissions();
    BluetoothConnection.toAddress(widget.server.address).then((_connection) {
      print('Connected to the device from app');
      connection = _connection;
      setState(() {
        isConnecting = false;
        isDisconnecting = false;
      });

      connection!.input?.listen(_onDataReceived).onDone(() {
        // Example: Detect which side closed the connection
        // There should be `isDisconnecting` flag to show are we are (locally)
        // in middle of disconnecting process, should be set before calling
        // `dispose`, `finish` or `close`, which all causes to disconnect.
        // If we except the disconnection, `onDone` should be fired as result.
        // If we didn't except this (no flag set), it means closing by remote.
        if (isDisconnecting) {
          print('Disconnecting locally!');
        } else {
          print('Disconnected remotely!');
        }
        if (mounted) {
          setState(() {});
        }
      });
    }).catchError((error) {
      print('app page Cannot connect, exception occured');
      print(error);
    });

    if (!hasPermission) {
      print("❌ Permissions NOT granted");
      return;
    }
  }

  Future<bool> requestPermissions() async {
    Map<Permission, PermissionStatus> statuses = await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.location,
    ].request();

    bool scanGranted =
        statuses[Permission.bluetoothScan] == PermissionStatus.granted;
    bool connectGranted =
        statuses[Permission.bluetoothConnect] == PermissionStatus.granted;
    bool locationGranted =
        statuses[Permission.location] == PermissionStatus.granted;

    print("Scan: $scanGranted");
    print("Connect: $connectGranted");
    print("Location: $locationGranted");

    return scanGranted && connectGranted;
  }

  @override
  void initState() {
    super.initState();
    _initBluetooth(); // start flow
  }

  @override
  void dispose() {
    // Avoid memory leak (`setState` after dispose) and disconnect
    if (isConnected) {
      isDisconnecting = true;
      connection!.dispose();
      connection = null;
    }

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final Text command = Text(message);

    return Scaffold(
      appBar: AppBar(title: Text("Raspberry Pi App")),
      body: SafeArea(
        child: Column(
          children: <Widget>[
            command,
            Expanded(
              flex: 3,
              child: MJPEGStreamScreen(
                streamUrl: 'http://10.54.1.111:5001/video_feed',
                showLiveIcon: true,
              ),
            ),
            ElevatedButton(
              child: const Text('Move Forward'),
              onPressed: () {
                _sendMessage("forward");
              },
            ),
            ElevatedButton(
              child: const Text('Move Backward'),
              onPressed: () {
                _sendMessage("backward");
              },
            ),
            ElevatedButton(
              child: const Text('Move Left'),
              onPressed: () {
                _sendMessage("left");
              },
            ),
            ElevatedButton(
              child: const Text('Move Right'),
              onPressed: () {
                _sendMessage("right");
              },
            ),
            ElevatedButton(
              child: const Text('stop'),
              onPressed: () {
                _sendMessage("stop");
              },
            )
          ],
        ),
      ),
    );
  }

  void _onDataReceived(Uint8List data) {
    // Allocate buffer for parsed data
    int backspacesCounter = 0;
    for (var byte in data) {
      if (byte == 8 || byte == 127) {
        backspacesCounter++;
      }
    }
    Uint8List buffer = Uint8List(data.length - backspacesCounter);
    int bufferIndex = buffer.length;

    // Apply backspace control character
    backspacesCounter = 0;
    for (int i = data.length - 1; i >= 0; i--) {
      if (data[i] == 8 || data[i] == 127) {
        backspacesCounter++;
      } else {
        if (backspacesCounter > 0) {
          backspacesCounter--;
        } else {
          buffer[--bufferIndex] = data[i];
        }
      }
    }

    // Create message if there is new line character
    String dataString = String.fromCharCodes(buffer);
    int index = buffer.indexOf(13);
    if (~index != 0) {
      setState(() {
        message = backspacesCounter > 0
            ? _messageBuffer.substring(
                0, _messageBuffer.length - backspacesCounter)
            : _messageBuffer + dataString.substring(0, index);

        _messageBuffer = dataString.substring(index);
      });
    } else {
      _messageBuffer = (backspacesCounter > 0
          ? _messageBuffer.substring(
              0, _messageBuffer.length - backspacesCounter)
          : _messageBuffer + dataString);
    }
  }

  // void _sendMessage(String text) async {
  //   text = text.trim();

  //   if (text.isNotEmpty) {
  //     try {
  //       connection!.output.add(Uint8List.fromList(utf8.encode(text)));
  //       await connection!.output.allSent;
  //     } catch (e) {
  //       // Ignore error, but notify state
  //       setState(() {});
  //     }
  //   }
  // }
  void _sendMessage(String text) async {
    text = text.trim();

    if (text.isNotEmpty) {
      if (connection == null) {
        print("No Bluetooth connection");
        return;
      }

      print("Sending: $text");

      try {
        connection!.output.add(Uint8List.fromList(utf8.encode("$text\n")));
        await connection!.output.allSent;
        print("Message sent");
      } catch (e) {
        print("Send error: $e");
        setState(() {});
      }
    }
  }
}
