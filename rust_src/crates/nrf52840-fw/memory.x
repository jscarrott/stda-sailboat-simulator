/* nRF52840: 1 MB flash, 256 KB RAM. No SoftDevice — bare-metal USB, so the
   application starts at the bottom of flash. (With a SoftDevice you would
   reserve its flash/RAM here instead.) */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
