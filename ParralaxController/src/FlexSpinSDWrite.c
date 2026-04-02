#include <Propellor/FlexSpinSDWrite.h>
#include <TankStatus/TankStatus.h>
#include <stdio.h>

// Standard Parallax Board of Education SD Card SPI Pins
/**
 * Call this ONCE in your main setup function to initialize the SD card
 * and write the column headers to the CSV file.
 */
static const char *filename = "DataLog.csv";

void initSDCardLogger() {
  // FlexSpin relies on standard C mounting if not using simpletools.
  // If you are using simpletools.h, you would call: sd_mount(SD_DO, SD_CLK,
  // SD_DI, SD_CS);

  // Open the file in "Append" mode ("a")
  // This creates the file if it doesn't exist, and adds to it if it does.
  FILE *fp = fopen(filename, "a");

  if (fp == NULL) {
    printf("ERROR: Could not mount SD card or open %s\n", filename);
    return;
  }

  // Write the CSV column headers
  fprintf(fp, "TimeStamp,DriveLeft,DriveRight,EulerX,EulerY,EulerZ,LockID,"
              "ChangeFlag\n");

  // Close the file to flush the buffer
  fclose(fp);
}

/**
 * Call this function whenever you want to log a frame of data.
 */
void asybcLogTankStatusToCSV(void *arg) {
  struct TankStatus *ts = (struct TankStatus *)arg;
  FILE *fp = fopen(filename, "a");

  if (fp == NULL) {
    return; // Fail silently if SD card is removed
  }

  // Write the struct data as comma-separated values
  // Using %d handles the type promotion for both unsigned char and short safely
  fprintf(fp, "%d,%d,%d,%d,%d,%d,%d\n", ts->driveLeft, ts->driveRight,
          ts->eulerX, ts->eulerY, ts->eulerZ, ts->lockID, ts->changeFlag);

  // VERY IMPORTANT: You must close the file to force the SPI
  // hardware to physically write the data cache to the flash memory!
  fclose(fp);
  ts->changeFlag = 0;
}
