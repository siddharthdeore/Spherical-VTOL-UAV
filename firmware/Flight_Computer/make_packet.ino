char Packet[12] = "";
short butn[12] = {
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

void sendPacket()
{
  btn = 0;
  btn += butn[11];
  btn += butn[10] * (2);
  btn += butn[9] * (4);
  btn += butn[8] * (8);
  btn += butn[7] * (16);
  btn += butn[6] * (32);
  btn += butn[5] * (64);
  btn += butn[4] * (128);
  btn += butn[3] * (256);
  btn += butn[2] * (512);
  btn += butn[1] * (1024);
  btn += butn[0] * (2048);
  Packet[0] = '#';

  Packet[1] = (x >> 6) % 64 + 48;
  Packet[2] = (x % 64) + 48;
  Packet[3] = (y >> 6) % 64 + 48;
  Packet[4] = (y % 64) + 48;
  Packet[5] = (z >> 6) % 64 + 48;
  Packet[6] = (z % 64) + 48;
  Packet[7] = (btn >> 6) % 64 + 48;
  Packet[8] = (btn % 64) + 48;

  Packet[9] = '$';
  for (int pu = 0; pu < 10; pu++)
  {
    HWS.print(Packet[pu]);
    delay(2);
  }
  HWS.println();
}
