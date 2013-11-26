#!/usr/bin/perl -w

$num=0; 

while(<STDIN>) { 
	chomp;
	@array = split(/\s/);
	$point1[$num][0] = $array[0];
	$point1[$num][1] = $array[1];
	$point1[$num][2] = $array[2];
	$num++;
}

print "VECT\n";
print "$num $num $num\n"; # the number of line segments, the number of vertices, the number of colors
for($i=0; $i<$num; $i++) {
	print "1 "; # the number tells how many vertices per line segment 
}
print "\n\n";
for($i=0; $i<$num; $i++) {
	print "1 "; # the number of colors per line segment
}
print "\n\n";

# printing the points
for($i=0; $i<$num; $i++) {
	print "$point1[$i][0] $point1[$i][1] $point1[$i][2]\n";
}


# printing the colors
for($i=0; $i<$num; $i++) { 
#	$r = rand();
#	$g = rand();
#	$b = rand(); 
	$r = 1; $g = 0; $b = 0; 
	
	print "$r $g $b 1\n";
}
