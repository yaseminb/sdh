function Tlw=LinkToWrist2_(theta)

Tlw=trans(19.053, 33, 25)*myrot(0, 0, -theta+pi)*myrot(pi/2, 0, pi/6);

end