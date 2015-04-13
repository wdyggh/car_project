
var STX = '\2', ETX = '\3';
const ACK = '8';	
const NACK = '9';
var auto_test_timer;
var wsUri = 'ws://220.69.240.54:8001';
var websocket;

var tx_protocol = 	{
						ID:'1', COMMAND:'1', SPEED:'I', DIR:'S', checksum:0
					};

var rx_protocol = 	{
						ID:'1', COMMAND:'1', STEP_CNT:0, ACK_NACK:'0', checksum:0
					};
					
$(document).ready(function() {
		
	/*
	// device check
	var device = check_device();
	if(device !=''){
		alert(device);
	}
	*/
	
	fun();
	
	// Not Drag
	
	$(document).bind("contextmenu",function(){return false;}); 
	$(document).bind("mousedown",function(){return false;});	
	
});

function avr_data_parsing(data) {
	var data_arr = new Array();
	data_arr = data.split(':');
	
	rx_protocol.ID = data_arr[1];
	rx_protocol.COMMAND = data_arr[2];
	rx_protocol.STEP_CNT = data_arr[3];
	rx_protocol.ACK_NACK = data_arr[4];
	rx_protocol.checksum = avr_check_sum_func(rx_protocol);	
	
	if ( rx_protocol.checksum == data_arr[5] ) {
		console.log( 'crc value Agreement');
		$("#step_cnt_"+rx_protocol.ID).val(rx_protocol.STEP_CNT);
	}


}

function testWebSocket()
{
	websocket = new WebSocket(wsUri);
	websocket.onopen = function(evt) { onOpen(evt); };
	websocket.onclose = function(evt) { onClose(evt); };
	websocket.onmessage = function(evt) { onMessage(evt); };
	websocket.onerror = function(evt) { onError(evt); };
}

function web_socket_init()
{
	testWebSocket();
}

function onOpen(evt)
{
	console.log('CONNECTED');
}

function onClose(evt)
{
	console.log('DISCONNECTED');
}

function onMessage(evt)
{	
	console.log('data : ' + evt.data);
	avr_data_parsing(evt.data);
}

function onError(evt)
{
	console.log('Error' + evt.data);
}

function avr_check_sum_func(protocol) {
	var buf = new Array(4);
	var string =  new Array();
	var i = 0, j = 0, crc = 0;
	var length_str_len;
	var colon = ':';
							
	string[i++] = STX.toString().charCodeAt(0);	// STX	'/2' 0x02
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.ID.toString().charCodeAt(0);		// NUM 	'1'	 0x31
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.COMMAND.toString().charCodeAt(0);	// COMMAND
	string[i++] = colon.toString().charCodeAt(0);
		
	for(var j=0; j< protocol.STEP_CNT.length; j++) {
		string[i++] = protocol.STEP_CNT.toString().charCodeAt(j);	
	}
	string[i++] = colon.toString().charCodeAt(0);
	
	string[i++] = protocol.ACK_NACK.toString().charCodeAt(0);	
	string[i++] = colon.toString().charCodeAt(0);
	
	for(var j = 0; j < i; j++) {
		crc = crc^string[j];	// LRC ..
	}
	
	
	// console.log('string length : ' + string.length );
	console.log('crc value: ' + crc);
	console.log('crc hex value: ' + dec2hex(crc));
	var crc_hex = dec2hex(crc);
			
	return crc_hex.slice(2);
}

function check_sum_func(protocol) {
			
	var buf = new Array(4);
	var string =  new Array();
	var i = 0, j = 0, crc = 0;
	var length_str_len;
	var colon = ':';
							
	string[i++] = STX.toString().charCodeAt(0);	// STX	'/2' 0x02
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.ID.toString().charCodeAt(0);		// NUM 	'1'	 0x31
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.COMMAND.toString().charCodeAt(0);	// COMMAND
	string[i++] = colon.toString().charCodeAt(0);
		
	string[i++] = protocol.SPEED.toString().charCodeAt(0);	
	string[i++] = colon.toString().charCodeAt(0);
	
	string[i++] = protocol.DIR.toString().charCodeAt(0);	
	string[i++] = colon.toString().charCodeAt(0);
	
	for(var j = 0; j < i; j++) {
		crc = crc^string[j];	// LRC ..
	}
	
	/*				
	console.log('result : '+ parseInt( dec2hex(protocol.STX.charCodeAt(0)) ));
	console.log('result : '+ parseInt( dec2hex(protocol.ID.charCodeAt(0)) ));
	console.log('result : '+ parseInt( dec2hex(protocol.COMMAND.charCodeAt(0)) ));
	*/
	
	// console.log('string length : ' + string.length );
	console.log('crc value: ' + crc);
	console.log('crc hex value: ' + dec2hex(crc));
	var crc_hex = dec2hex(crc);
			
	return crc_hex.slice(2);
	
}

function dec2hex(i)
{
	var result = "0000";
	if      (i >= 0    && i <= 15)    { result = "000" + i.toString(16).toUpperCase(); }
	else if (i >= 16   && i <= 255)   { result = "00"  + i.toString(16).toUpperCase(); }
	else if (i >= 256  && i <= 4095)  { result = "0"   + i.toString(16).toUpperCase(); }
	else if (i >= 4096 && i <= 65535) { result =         i.toString(16).toUpperCase(); }
	return result;
}

function make_tx_protocol() {
	tx_protocol.checksum = check_sum_func(tx_protocol);	
	return STX+':'+tx_protocol.ID+':'+tx_protocol.COMMAND+':'+tx_protocol.SPEED+':'+tx_protocol.DIR+':'+tx_protocol.checksum+':'+ETX+':'+'\0';
}

function fun()
{	
	web_socket_init();
	
	$(".foward").click(function() {
		console.log('foward');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'F'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
		
	});
	
	$(".stop").click(function() {
		console.log('stop');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'S'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	});
	
	$(".speed_down").click(function() {
		console.log('speed_down');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		tx_protocol.SPEED = 'D';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	});
	
	$(".speed_up").click(function() {
		console.log('speed_up');
		console.log('speed_down');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		tx_protocol.SPEED = 'U';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	});
	
	var auto_id = 1;
	$("#auto_test").click(function() {
		console.log('auto_test');
		
		auto_test_timer = setInterval(function() {
			
			if( auto_id > 4 ) auto_id = 1;
			
			tx_protocol.ID = (auto_id++).toString();
			tx_protocol.COMMAND = '1';
			tx_protocol.SPEED = 'I';
			// tx_protocol.DIR = 'S';
			var tx_send_data = make_tx_protocol();
			websocket.send(tx_send_data);
			
		} , 250);	
	});
	
	$("#auto_stop").click(function() {
		console.log('auto_stop');
		
		auto_id = 1;
		clearInterval(auto_test_timer );
	});
}