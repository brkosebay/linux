/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SPI_APPLE_S8000_PHASE_H
#define SPI_APPLE_S8000_PHASE_H

/*
 * AppleSamsungSPI 0xfffffff005b47a44: clock-phase bit 0 -> CFG[1];
 * bit 1 -> CFG[20] and WORD_DELAY >= 2 on spi-version 1. Mesa sets
 * phase=3 in 0xfffffff005ef9fd4. Bit 1 is not Linux SPI_CPOL.
 * The electrical meaning of CFG[20] is not established; preserve the
 * recovered programming without inventing a sampling/edge interpretation.
 * Call only for the optional apple,s8000-spi-clock-phase board property.
 */
static inline int s8000_spi_apply_phase(u32 mode, u32 phase, u32 *cfg,
				       u64 *word_delay_cycles)
{
	if (phase > 3 || !!(mode & SPI_CPHA) != !!(phase & 1))
		return -EINVAL;
	if (phase & 2) {
		*cfg |= BIT(20);
		if (*word_delay_cycles < 2)
			*word_delay_cycles = 2;
	}
	return 0;
}

#endif
