
var path = require('path');
var net = require('net');
var ws = require('nodejs-websocket');

var access_cnt=0;
//var PORT = 9000, HOST = '220.68.139.15';
var PORT = 9000, HOST = '220.69.240.54';

var netsocket = new Array(); //netsocket
var ws_sock = new Array();   //web socket

//var adc_data = { registor: 0, gas: 0, temp: 0 };
var STX = 0x02, ETX = 0x03;


var server = ws.createServer(function (conn) {
    ws_sock.push(conn);
	
	console.log("New connection");
	//conn.sendText(adc_data.registor);
	
    conn.on("text", function (str) {
        if( netsocket.length > 0 ) {
			netsocket[0].write(str);
			console.log('web data: '+ str);	
		}
    });
    conn.on("close", function (code, reason) {
        console.log("Connection closed");
		var i = ws_sock.indexOf(conn);
		ws_sock.splice(i,1);
    });
	
}).listen(8001);

// Create a server instance, and chain the listen function to it
// The function passed to net.createServer() becomes the event handler for the 'connection' event
// The sock object the callback function receives UNIQUE for each connection
net.createServer( 'connection', function(sock) {
	
	// sock.setNoDelay(true); 
	netsocket.push(sock);
	
	// We have a connection - a socket object is assigned to the connection automatically
	console.log('CONNECTED: ' + sock.remoteAddress +':'+ sock.remotePort);
	
	// Add a 'data' event handler to this instance of socket
	// TCP Client 에서 data 를 받을 경우. 이벤트 연결
	sock.on('data', function(data) {
		
		console.log('DATA ' + sock.remoteAddress + ': ' + data);
		// adc_data.registor = data;
		// process.emit('adc_data', data);		// web_sock 이벤트 발생.
		if( ws_sock.length > 0 ) {
			var str = JSON.stringify(data);
			//console.log(str);
			//console.log(str.indexOf(ETX));			
						
			/*
			console.log(' data type is ' + typeof(data)); 
			if( data.toString() == ETX ) {
				console.log('data is ETX');
			}
			*/
			ws_sock[0].sendText(data);
		}	
						
	});
	
	sock.on('uncaughtException', function (error) {
		console.error(err.stack);
		console.log("Node NOT Exiting...")
		console.log('Caught exception: ' + error);
	});
	
	sock.on('error', function(err) {
		console.log(err);
	});
	
	// Add a 'close' event handler to this instance of socket
	sock.on('close', function(sock) {
	
		console.log('Connection closed');
			
		var i = netsocket.indexOf(sock);
		netsocket.splice(i,1);
		
		
	});

}).listen(PORT, HOST, function() {
	console.log('Server listening on ' + HOST +':'+ PORT);
});

/*
var express = require('express');
var path = require('path');
var favicon = require('serve-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');

// uncomment after placing your favicon in /public
//app.use(favicon(__dirname + '/public/favicon.ico'));
app.use(logger('dev'));
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', routes);
app.use('/users', users);

// catch 404 and forward to error handler
app.use(function(req, res, next) {
    var err = new Error('Not Found');
    err.status = 404;
    next(err);
});

// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
    app.use(function(err, req, res, next) {
        res.status(err.status || 500);
        res.render('error', {
            message: err.message,
            error: err
        });
    });
}

// production error handler
// no stacktraces leaked to user
app.use(function(err, req, res, next) {
    res.status(err.status || 500);
    res.render('error', {
        message: err.message,
        error: {}
    });
});


module.exports = app;
*/
