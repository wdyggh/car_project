
var STX = '\2', ETX = '\3';
const ACK = '8';	
const NACK = '9';
var auto_test_timer;
//var wsUri = 'ws://220.69.240.54:8001';
var wsUri = 'ws://220.68.139.15:8001';

var websocket;
var auto_id = 1;
var tx_command = { step_cnt_read:'1', stet_cnt_write:'2' };

var tx_states = {
					step_cnt_read:0, another_cmd:1, nack:2
				};
				
var tx_state = tx_states.step_cnt_read;	
				
var car_count = 	{
						car_1: 0, car_2: 0, 
						car_3: 0, car_4: 0
					};

					var tx_protocol = 	{
						ID:'1', COMMAND:'1', DIR:'S', checksum:0
					};

var rx_protocol = 	{
						ID:'1', COMMAND:'1', STEP_CNT:0, ACK_NACK:'0', checksum:0
					};

var car_img_arr = [ "./images/car1.png", "./images/car2.png", "./images/car3.png", "./images/car4.png"];
var door_img = [ "./images/door1.png", "./images/door2.png" ];



var canvas_t;	// canvas timer
var car_pos = 	{	 
					x_1: 0, x_2: 0, x_3: 0, x_4: 0,
					y_1: 0, y_2: 0, y_3: 0, y_4: 0
				};
var door_pos = 	{
					x_1:0 , x_2:0,
					y_1:0 , y_2:0
				}
				
				
// Now you can just call
var ctx;
var door = new Array();
var car_img = new Array();

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
	
	rx_protocol.ID = data_arr[1];
	rx_protocol.COMMAND = data_arr[2];
	rx_protocol.STEP_CNT = data_arr[3];
	rx_protocol.ACK_NACK = data_arr[4];
	rx_protocol.checksum = avr_check_sum_func(rx_protocol);	
		
	if ( rx_protocol.checksum == data_arr[5] ) {
		return 1;
	} else {
		return 0;
	}	
		
}

function auto_tx_func() {
	
	if( auto_id > 2 ) auto_id = 1;
	
	tx_protocol.ID = (auto_id++).toString();
	tx_protocol.COMMAND = tx_command.step_cnt_read;
	tx_protocol.DIR = 'N';
	var tx_send_data = make_tx_protocol();
	websocket.send(tx_send_data);
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
		
		console.log( 'crc value Agreement');
		car_count['car_'+rx_protocol.ID] = h2d(rx_protocol.STEP_CNT);
		car_count['car_'+rx_protocol.ID] = parseInt((car_count['car_'+rx_protocol.ID]/166.6666666667), 10);
		
		$("#step_cnt_"+rx_protocol.ID).val(car_count['car_'+rx_protocol.ID]);
		
		if( rx_protocol.ACK_NACK == '8') {
			$("#ack_nack_"+rx_protocol.ID).val('ACK!');
		} else {
			$("#ack_nack_"+rx_protocol.ID).val('NACK!');
		}
		
	}
	
	if( rx_protocol.ID == '2' && rx_protocol.COMMAND == '1' ) {
		
		clearInterval(auto_test_timer);
										
		// COMMAND 2 : tx_command.stet_cnt_write
		make_tx_protocol(tx_command.stet_cnt_write);
		
		// AckNack Value INIT
		for(var i=1; i<5; i++) {
			$("#ack_nack_"+i).val('');
		}
	
		auto_test_timer = setInterval( auto_tx_func, 100);	
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
							
	string[i++] = STX.toString().charCodeAt(0);		// STX	'/2' 0x02
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.COMMAND.toString().charCodeAt(0); // COMMAND
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.ID.toString().charCodeAt(0);
	string[i++] = colon.toString().charCodeAt(0);
	
	// Command '1'
	if( protocol.COMMAND == '1' )  {
		for(var j=0; j< protocol.STEP_CNT.length; j++) {
			string[i++] = protocol.STEP_CNT.toString().charCodeAt(j);	
		}
		string[i++] = colon.toString().charCodeAt(0);
	}
	
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
	string[i++] = protocol.COMMAND.toString().charCodeAt(0);	// COMMAND
	string[i++] = colon.toString().charCodeAt(0);
	string[i++] = protocol.ID.toString().charCodeAt(0);		// NUM 	'1'	 0x31
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

function make_tx_protocol() {
	
	var tx_protocol_string;
	
	switch(tx_protocol.COMMAND) {
		
		case tx_command.step_cnt_read  : 	
					tx_protocol.checksum = check_sum_func(tx_protocol);	// check_sum store
					tx_protocol_string = STX+':'+tx_protocol.COMMAND+':'+tx_protocol.ID
					+':'+tx_protocol.DIR+':'+tx_protocol.checksum+':'+ETX+':'+'\0';
					break;
		
		case tx_command.stet_cnt_write : 	
					tx_protocol_string = STX+':'+tx_protocol.COMMAND+':'
					+'1'+':'+car_count.car_1+':'+'2'+':'+car_count.car_2
					+':'+'3'+':'+car_count.car_3+':'+'4'+':'+car_count.car_4;
					
					tx_protocol.checksum = check_sum_func(tx_protocol);	// check_sum store
					tx_protocol_string += ':'+tx_protocol.checksum+':'+ETX+':'+'\0';
					break;
					
		default: 	tx_protocol.checksum = check_sum_func(tx_protocol);	// check_sum store
					tx_protocol_string = STX+':'+tx_protocol.COMMAND
					+':'+tx_protocol.ID+':'+tx_protocol.DIR+':'
					+tx_protocol.checksum+':'+ETX+':'+'\0';
					break;
					
	}
	
	return tx_protocol_string;
}

function background_img_draw() {
	ctx.roundRect(80, 50, 1300, 500);
	ctx.roundRect(500, 50, 500, 95);
}
								
// image init
function canvas_img_init() {
	
	canvas = document.getElementById("mycanvas");
			
	door[0] = new Image();
	door[1] = new Image();
	
	door_pos.x_1 = 400;
	door_pos.y_1 = 0;
	
	door_pos.x_2 = 1020;
	door_pos.y_2 = 0;

	door[0].src = door_img[0];
	door[1].src = door_img[1];
	
	car_img[0] = new Image();	
	car_img[1] = new Image();
	car_img[2] = new Image();
	car_img[3] = new Image();
	
	car_pos.x_1 = 1100;
	car_pos.y_1 = 30;
	
	car_pos.x_2 = 50;
	car_pos.y_2 = 530;
	
	car_pos.x_3 = 50;
	car_pos.y_3 = 530;
	
	
	car_pos.x_4 = 50;
	car_pos.y_4 = 530;
	
	car_img[0].src = car_img_arr[0];
	car_img[1].src = car_img_arr[1];
	car_img[2].src = car_img_arr[2];
	car_img[3].src = car_img_arr[3];

}

function door_img_draw() {
	ctx.drawImage(door[0], door_pos.x_1, door_pos.y_1);
	ctx.drawImage(door[1], door_pos.x_2, door_pos.y_2);
}

function car_img_draw(car_pos_obj, car_id, car_img_arr) {
		
	// console.log('car_count[car_'+car_id+'] : '+ car_count['car_'+car_id]);
	if( (car_count['car_'+car_id] >= 0) && (car_count['car_'+car_id] <= 280) ) {

		car_pos_obj['x_'+car_id] = 330-(car_count['car_'+car_id]-0);

	} else if( (car_count['car_'+car_id] >= 280) && (car_count['car_'+car_id] <= 780) ) {
	
		car_pos_obj['x_'+car_id] = 50;
		car_pos_obj['y_'+car_id] = 30+(car_count['car_'+car_id]-280);
	
	} else if( (car_count['car_'+car_id] >= 780) && (car_count['car_'+car_id] <= 2080) ) {
	
		car_pos_obj['y_'+car_id] = 530;
		car_pos_obj['x_'+car_id] = 50+(car_count['car_'+car_id]-780);
	
	} else if( (car_count['car_'+car_id] >= 2080) && (car_count['car_'+car_id] <= 2580) ) {
		
		car_pos_obj['x_'+car_id] = 1350;
		car_pos_obj['y_'+car_id] = 530-(car_count['car_'+car_id]-2080);
		
	} else if( (car_count['car_'+car_id] >= 2580) && (car_count['car_'+car_id] <= 2830) ) {
		
		car_pos_obj['y_'+car_id] = 30;
		car_pos_obj['x_'+car_id] = 1350-(car_count['car_'+car_id]-2580);
		
	} else if( (car_count['car_'+car_id] >= 2830) && (car_count['car_'+car_id] <= 3600) ) {
		car_pos_obj['y_'+car_id] = 30;
		car_pos_obj['x_'+car_id] = 1100-(car_count['car_'+car_id]-2830);
	}
	
	ctx.drawImage(car_img_arr, car_pos_obj['x_'+car_id], car_pos_obj['y_'+car_id], 50, 50);
	
}

function car_img_draw_old(car_pos_obj, car_id, car_img_arr) {
	
	if( car_pos_obj['x_'+car_id] < 1350 && car_pos_obj['y_'+car_id]==530) {	// x1~x2 end, y2
		car_pos_obj['x_'+car_id]+=10;
		// console.log("car_pos_x+=10");
	} else if( car_pos_obj['x_'+car_id] >= 1350 && car_pos_obj['y_'+car_id] > 30 ) {
		car_pos_obj['x_'+car_id] = 1350; 
		// console.log("car_pos_x = 1350");
		
		if ( car_pos_obj['y_'+car_id] <= 530 && car_pos_obj['y_'+car_id] > 30) {	// x2, y2~y1
			car_pos_obj['y_'+car_id]-=10;		
			// console.log("car_pos_y-=30");
		} else if ( car_pos_obj['y_'+car_id] <= 30 ) {
			
			car_pos_obj['y_'+car_id] = 30;
			// console.log("car_pos_y = 30");
		}
		
	} else if( car_pos_obj['x_'+car_id] <= 1350 && car_pos_obj['y_'+car_id] <= 30 && car_pos_obj['x_'+car_id] > 50) {	// x2~x1, y1
		
		car_pos_obj['x_'+car_id]-=10;
		
		// door 2 open: front
		if ( car_pos_obj['x_'+car_id] > 950 && car_pos_obj['x_'+car_id] < 1150 ) {
			door_pos.y_2 = 200;
		} else {
			door_pos.y_2 = 0;
		}
		
		// door 1 open: rear
		if ( car_pos_obj['x_'+car_id] > 300 && car_pos_obj['x_'+car_id] < 500 ) {
			door_pos.y_1 = 200;
		} else {
			door_pos.y_1 = 0;
		}
		
		if ( car_pos_obj['x_'+car_id] <= 50 ) {
			car_pos_obj['x_'+car_id]=50;
						
			// console.log("car_pos_x=50");
		}
		
	} else if( car_pos_obj['x_'+car_id] <= 50 && car_pos_obj['y_'+car_id] >= 30 ){	// x1, y1~y2
		
		car_pos_obj['y_'+car_id]+=10;
		if ( car_pos_obj['y_'+car_id] >= 530 ) {
			car_pos_obj['y_'+car_id]=530;
		}
		//console.log("else");
		
	}
			
	ctx.drawImage(car_img_arr, car_pos_obj['x_'+car_id], car_pos_obj['y_'+car_id], 50, 50);

}

function canvasApp() {
	
	ctx.clearRect(0,0,canvas.width,canvas.height);
		
	ctx.save();
	
	background_img_draw();
	door_img_draw();
	
	// PARAMETER: car_pos obj, car ID, car image array
	car_img_draw(car_pos, '1', car_img[0]);	
	//ctx.drawImage(car_img[0],car_pos.x_1,car_pos.y_1,50,50);
	
	/*
	car_img_draw(car_pos, '2', car_img[1]);	
	car_img_draw(car_pos, '3', car_img[2]);	
	car_img_draw(car_pos, '4', car_img[3]);	
	*/
	ctx.restore();

}

function fun()
{	
	web_socket_init();
	ctx = document.getElementById("mycanvas").getContext("2d");	
	canvas_img_init();
		
	(function() {
		canvas_t = setInterval(canvasApp, 10);
	}());
	
	
	$(".specific_location_1").click(function() {
				
	});
	
	$(".specific_location_2").click(function() {
				
	});
	
	$(".specific_location_3").click(function() {
				
	});
	
	$(".specific_location_4").click(function() {
				
	});
		
	$("#auto_test").click(function() {
		console.log('auto_test');
		
		auto_test_timer = setInterval( auto_tx_func, 200);	
	});
	
	$("#auto_stop").click(function() {
		console.log('auto_stop');
		
		auto_id = 1;
		clearInterval(auto_test_timer);
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