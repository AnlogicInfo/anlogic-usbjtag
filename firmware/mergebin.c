/* This file is licensed by GPLv3 */
/* For combining bootloader and firmware */
/* Zhiyuan Wan 2017 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

int main()
{
	FILE *fp1, *fp2, *fp3;
	int bsz, fsz;
	fp1 = fopen("bootloader.fwb", "rb");
	fp2 = fopen("firmware.fwb", "rb");
	fp3 = fopen("flash.fwb", "wb+");
	
	if(!fp1 || !fp2 || !fp3)
	{
		fprintf(stderr, "I/O error!\n");
		return -1;
	}
	
	fseek(fp1, 0, SEEK_END);
	fseek(fp2, 0, SEEK_END);
	bsz = ftell(fp1);
	fsz = ftell(fp2);
	fseek(fp1, 0, SEEK_SET);
	fseek(fp2, 0, SEEK_SET);
	
	printf("bootloader size:%d\n", bsz);
	printf("firmware size: %d\n", fsz);
	
	if(bsz > 8192 || (bsz+fsz) > 65536)
	{
		fprintf(stderr, "Bootloader is out of memory or firmware is out of memory!\n");
		return -1;
	}
	printf("flash size: %d\n", bsz+fsz);
	
	uint8_t *buffer = malloc(8192 + fsz);
	
	memset(buffer, 0xff, bsz+fsz);
	
	fread(buffer, 1, bsz, fp1);
	fread(&buffer[8192], 1, fsz, fp2);
	
	fwrite(buffer, 1, 8192 + fsz, fp3);
	
	printf("firmware written to flash.fwb, You can use STVP or stm32flash to flash it in your microcontroller!\n");
	
	free(buffer);
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
}
