
var STX = '\2', ETX = '\3';
const ACK = '8';	
const NACK = '9';
var auto_test_timer;
//var wsUri = 'ws://220.69.240.54:8001';
var wsUri = 'ws://220.69.240.4:8001';
var websocket;

var car_count = 	{
						car_1: 0, car_2: 0, 
						car_3: 0, car_4: 0
					};
					
var nearing_level = {
						LONG:600, MIDDLE:300, SHORT:100, STOP:50
					};
				
var tx_protocol = 	{
						//ID:'1', COMMAND:'1', SPEED:'N', DIR:'S', checksum:0
						COMMAND:'1',ID:'1', DIR:'S', checksum:0
					};

var rx_protocol = 	{
						//ID:'1', COMMAND:'1', STEP_CNT:0, ACK_NACK:'0', checksum:0
						COMMAND:'1',ID:'1', STEP_CNT:0, ACK_NACK:'0', checksum:0
					};

var car_img_arr = [ "./images/car1.png", "./images/car2.png", "./images/car3.png", "./images/car4.png"];
var door_img = [ "./images/door1.png", "./images/door2.png" ];

CanvasRenderingContext2D.prototype.roundRect = function(x, y, width, height, radius, fill, stroke) {
	if (typeof stroke == "undefined" ) {
		stroke = true;
	}
	if (typeof radius === "undefined") {
		radius = 50;
	}
	this.beginPath();
	this.moveTo(x + radius, y);
	this.lineTo(x + width - radius, y);
	this.quadraticCurveTo(x + width, y, x + width, y + radius);
	this.lineTo(x + width, y + height - radius);
	this.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
	this.lineTo(x + radius, y + height);
	this.quadraticCurveTo(x, y + height, x, y + height - radius);
	this.lineTo(x, y + radius);
	this.quadraticCurveTo(x, y, x + radius, y);
	this.closePath();
	if (stroke) {
		this.stroke();
	}
	if (fill) {
		this.fill();
	}        
}		

function position_calc_cmd(lv_id, ID) {
	
	if( lv_id > nearing_level.LONG ) {
		tx_protocol.ID = ID;
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'D';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	}
	else if( (lv_id > (nearing_level.MIDDLE+100)) && (lv_id < (nearing_level.LONG+100)) ) {	// 400 < < 700
		tx_protocol.ID = ID;
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'D';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	} else if ( (lv_id > nearing_level.SHORT+100) && (lv_id < nearing_level.MIDDLE+100) ) {	// 200 < < 400
		tx_protocol.ID = ID;
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	} else if ( (lv_id > nearing_level.STOP+100) && (lv_id < nearing_level.SHORT+100) ) {		// 150 < < 200
		tx_protocol.ID = ID;
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'U';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	} else if ( lv_id <= nearing_level.STOP ) {
		tx_protocol.ID = ID;
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'S'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	} else {
		tx_protocol.ID = ID;
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'N'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	}
}

function position_calc(car_pos) {
	
	var nearLv_1_2 = car_pos['car_1'] - car_pos['car_2'];
	var nearLv_2_3 = car_pos['car_2'] - car_pos['car_3'];
	var nearLv_3_4 = car_pos['car_3'] - car_pos['car_4'];
	
	position_calc_cmd(nearLv_1_2, '2');
	position_calc_cmd(nearLv_2_3, '3');
	position_calc_cmd(nearLv_3_4, '4');
	
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

function h2d(h) {return parseInt(h,16);}

function avr_data_parsing(data) {
	
	var data_arr = new Array();
	data_arr = data.split(':');
	
	//rx_protocol.ID = data_arr[1];
	rx_protocol.COMMAND = data_arr[1];
	//rx_protocol.COMMAND = data_arr[2];
	rx_protocol.ID = data_arr[2];
	rx_protocol.STEP_CNT = data_arr[3];
	rx_protocol.ACK_NACK = data_arr[4];
	rx_protocol.checksum = avr_check_sum_func(rx_protocol);	
		
	if ( rx_protocol.checksum == data_arr[5] ) {
		return 1;
	} else {
		return 0;
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
	var parse_result;
	
	console.log('data : ' + evt.data);
	parse_result = avr_data_parsing(evt.data);
	
	if( parse_result == 1 ) {
		car_count['car_'+rx_protocol.ID] = rx_protocol.STEP_CNT;
		console.log( 'crc value Agreement');
		$("#step_cnt_"+rx_protocol.ID).val(h2d(car_count['car_'+rx_protocol.ID]));
	}
	
	if( rx_protocol.ID == '4' ) {
		
		position_calc(car_count);
		/*
		for( var key in car_count ) {
			console.log(key +' : ' + car_count[key]);
		}
		*/
	}
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
//	string[i++] = protocol.ID.toString().charCodeAt(0);		// NUM 	'1'	 0x31
	string[i++] = protocol.COMMAND.toString().charCodeAt(0);		//COMMAND 
	string[i++] = colon.toString().charCodeAt(0);
//	string[i++] = protocol.COMMAND.toString().charCodeAt(0);	// COMMAND
	string[i++] = protocol.ID.toString().charCodeAt(0);	// NUM 	'1'	 0x31
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
//	string[i++] = protocol.ID.toString().charCodeAt(0);		// NUM 	'1'	 0x31
	string[i++] = protocol.COMMAND.toString().charCodeAt(0);		//  COMMAND
	string[i++] = colon.toString().charCodeAt(0);
//	string[i++] = protocol.COMMAND.toString().charCodeAt(0);	// COMMAND
	string[i++] = protocol.ID.toString().charCodeAt(0);	//NUM 	'1'	 0x31
	string[i++] = colon.toString().charCodeAt(0);
		
//	string[i++] = protocol.SPEED.toString().charCodeAt(0);	
//	string[i++] = colon.toString().charCodeAt(0);
	
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

function make_tx_protocol() {
	tx_protocol.checksum = check_sum_func(tx_protocol);	
	return STX+':'+tx_protocol.COMMAND+':'+tx_protocol.ID+':'+tx_protocol.DIR+':'+tx_protocol.checksum+':'+ETX+':'+'\0';
	//return STX+':'+tx_protocol.ID+':'+tx_protocol.COMMAND+':'+tx_protocol.SPEED+':'+tx_protocol.DIR+':'+tx_protocol.checksum+':'+ETX+':'+'\0';
	//return STX+':'+tx_protocol.ID+':'+tx_protocol.COMMAND+':'+tx_protocol.SPEED+':'+tx_protocol.DIR+':'+tx_protocol.checksum+':'+ETX+':'+'\0';
}

function fun()
{	
	web_socket_init();
	// Now you can just call
	var ctx = document.getElementById("mycanvas").getContext("2d");
	
	ctx.roundRect(80, 50, 1300, 500);
	ctx.roundRect(500, 340, 500, 200);
	
	var door = new Array();
	door[0] = new Image();
	door[1] = new Image();
		
	door[0].onload = function() {
		ctx.drawImage(door[0], 400, 400);
	};
	door[0].src = door_img[0];
	
	door[1].onload = function() {
		ctx.drawImage(door[1], 1020, 400);
	};
	door[1].src = door_img[1];

	var car_img = new Array();
	car_img[0] = new Image();
	car_img[1] = new Image();
	car_img[2] = new Image();
	car_img[3] = new Image();
	
	car_img[0].onload = function() {
		ctx.drawImage(car_img[0], 60, 400, 50, 50);
	};
	car_img[0].src = car_img_arr[0];
	
	car_img[1].onload = function() {
		ctx.drawImage(car_img[1], 60, 340, 50, 50);
	};
	car_img[1].src = car_img_arr[1];
	
	car_img[2].onload = function() {
		ctx.drawImage(car_img[2], 60, 280, 50, 50);
	};
	car_img[2].src = car_img_arr[2];
	
	car_img[3].onload = function() {
		ctx.drawImage(car_img[3], 60, 220, 50, 50);
	};
	car_img[3].src = car_img_arr[3];
	
	$(".foward").click(function() {
		console.log('foward');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'F'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
		
	});
	
	$(".stop").click(function() {
		console.log('stop');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'N';
		tx_protocol.DIR = 'S'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	});
	
	$(".speed_down").click(function() {
		console.log('speed_down');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'D';
		tx_protocol.DIR = 'D'
		var tx_send_data = make_tx_protocol();
		websocket.send(tx_send_data);
	});
	
	$(".speed_up").click(function() {
		console.log('speed_up');
		console.log('speed_down');
		
		tx_protocol.ID = this.id.charAt(2);
		tx_protocol.COMMAND = '1';
		//tx_protocol.SPEED = 'U';
		tx_protocol.DIR = 'U'
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
			//tx_protocol.SPEED = 'N';
			tx_protocol.DIR = 'F';
			var tx_send_data = make_tx_protocol();
			websocket.send(tx_send_data);
			
		} , 20);	
	});
	
	$("#auto_stop").click(function() {
		console.log('auto_stop');
		
		auto_id = 1;
		clearInterval(auto_test_timer );
	});
}

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