let angle = 0;

while (true) {
  console.log("pitch:", 90 + 10 * Math.sin(angle));
  console.log("yaw:", 90 + 10 * Math.cos(angle));

  angle += 1;
}
