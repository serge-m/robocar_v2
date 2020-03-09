import struct


# def getMouseEvent():
#   getMouseEvent();
  # return stuffs
sum_X = 0;
# while( 1 ):
for cc1 in range(500):
	# file = open( "/dev/input/mice", "rb" );
	file = open( "/dev/input/mouse0", "rb" );
	buf = file.read(3);
	file.close();
	# print(buf);
	# button = ord( buf[0] );
	button = buf[0];
	bLeft = button & 0x1;
	bMiddle = ( button & 0x4 ) > 0;
	bRight = ( button & 0x2 ) > 0;
	x,y = struct.unpack( "bb", buf[1:] );
	sum_X = sum_X + x;
	print ("cc1:%d, L:%d, M: %d, R: %d, x: %d, y: %d, === sum_X: %d \n" % (cc1,bLeft,bMiddle,bRight, x, y, sum_X) );
