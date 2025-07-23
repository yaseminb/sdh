function Tlw=LinkToWristv_(theta)

Tlw=trans(-((33/2)*sqrt(3)-19.053), 33/2, 25)*myrot(0, 0, theta+pi)*myrot(pi/2, 0, pi/6);

end