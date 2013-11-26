#!/usr/bin/perl 

$num=0; 

$coff1 = $ARGV[0]; $coff1 = "temp-results/colorMesh.20.off" if($coff1 eq '');
$coff2 = $ARGV[1]; $coff2 = "temp-results/colorMesh.40.off" if($coff2 eq '');
$vects = $ARGV[2]; $vects = "vertexDeltas.20.mat" if($vects eq '');
$apart = $ARGV[3]; $apart = 1.2 if($apart eq '');
$apartDim=$ARGV[4]; $apartDim = 2 if($apartDim eq '' || $apartDim > 2);

print STDERR "from($coff1) to($coff2) vects($vects) apart($apart) in dimension($apartDim) \n";

open(VECTS,$vects) or die "Couldn't open the vects-file $vects";
open(COFF1,$coff1) or die "Couldn't open coff1-file $coff1";
open(COFF2,$coff2) or die "Couldn't open coff2-file $coff2";

print "{ # World \n";
print "  INST \n";
print "  geom { LIST # World list \n";
print "    { # before-model($coff1) \n";
print "      INST \n";
print "      geom\t appearance { \n";
print "        face \n";
print "      }\n";
print "      { # copying the geometry \n";
while(<COFF1>) { 
$line=$_; chomp($line);
print "        $line\n";
}
close(COFF1);
print "      } \n";
print "    } # end <geom and INST> before-model($coff1) \n";
print "    { # after-model($coff2) \n";
print "      INST \n";
print "      geom\t appearance { \n";
print "        face \n";
print "      }\n";
print "      { # copying the geometry \n";
$line = <COFF2>; chomp($line);
print "        $line\n"; # says COFF
$line = <COFF2>; chomp($line); $line =~ s/^\s+//;
($numVert2, $numSurf2, $numEdge2) = split(/\s+/, $line);
#print STDERR "COFF2 :: $numVert2, $numSurf2, $numEdge2";
print "        $line\n";
for($i=0; $i<$numVert2; $i++) { 
$line = <COFF2>; chomp($line); $line =~ s/^\s+//; 
@vattrib = split(/\s+/, $line);
print "        ";
for($j=0;$j<$apartDim;$j++) {
	print $vattrib[$j]." ";
}
for(;$j<$apartDim+1;$j++) {
	print $vattrib[$j]+$apart." ";
#	print STDERR $vattrib[$j]." to ";
#	print STDERR $vattrib[$j]+$apart." ";
}
for(;$j<3;$j++) {
	print $vattrib[$j]." ";
}
#print STDERR "\n";
for(;$j<7;$j++) {
	print $vattrib[$j]." ";
}
print "\n";
}
while(<COFF2>) { 
$line=$_; chomp($line); $line =~ s/^\s+//;
print "        $line\n";
}
close(COFF2);
print "      }\n";
print "    } # end <geom and INST> after-model($coff2) \n";
print "    { # vects ($vects) \n";
print "      INST \n";
print "      geom\t appearance { \n";
print "        linewidth 3\n";
print "      } \n";
print "      { # copying the geometry \n";
while(<VECTS>) { 
	chomp;
	@array = split(/\s/);
	$point1[$num][0] = $array[1];
	$point1[$num][1] = $array[2];
	$point1[$num][2] = $array[3];
	$point2[$num][0] = $array[4];
	$point2[$num][1] = $array[5];
	$point2[$num][2] = $array[6];
	#if($point1[$num][0] != $point2[$num][0] && $point1[$num][1] != $point2[$num][1] && $point1[$num][2] != $point2[$num][2] ) { # Comment this line if you want all the points printed 
		# $point2[$num][0] += $apart;
		# $point2[$num][1] += $apart;
		$point2[$num][$apartDim] += $apart;
		$num++; 
	#}# Comment this line if you want all the points printed 
}
close(VECTS);
print "        VECT\n";
print "        ".$num." ".2*$num." $num\n";
print "        ";
for($i=0; $i<$num; $i++) {
	print "2 ";
}
print "\n\n";
print "        ";
for($i=0; $i<$num; $i++) {
	print "1 ";
}
print "\n\n";

# printing the points
for($i=0; $i<$num; $i++) {
	print "        $point1[$i][0] $point1[$i][1] $point1[$i][2]\n";
	print "        $point2[$i][0] $point2[$i][1] $point2[$i][2]\n";
}

# printing the colors
for($i=0; $i<$num; $i++) { 
#	$r = rand(); $g = rand(); $b = rand(); 
	$r = $i/$num;
	if(2*$i < $num) { 
		$g = 2*$i/$num;
	} else { 
		$g = 2*($num-$i)/$num;
	}
	$b = 1 - $i/$num;

#	$r = 0.9; $g = 0.4; $b = 0.05;
	print "        $r $g $b 1\n";
}

print "      } \n";
print "    } # end <geom and INST> vects\n";
print "  } # end World list\n";
print "} # end World \n";

