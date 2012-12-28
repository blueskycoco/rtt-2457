/*
 * @brief LPC18xx/43xx clock driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "clock_18xx_43xx.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Maps a peripheral clock to it's base clock */
typedef struct {
	CCU_CLK_T clkstart;
	CCU_CLK_T clkend;
	CGU_BASE_CLK_T clkbase;
} CLK_PERIPH_TO_BASE_T;
static const CLK_PERIPH_TO_BASE_T periph_to_base[] = {
	{CLK_APB3_BUS, CLK_APB3_CAN0, CLK_BASE_APB3},
	{CLK_APB1_BUS, CLK_APB1_CAN1, CLK_BASE_APB1},
	{CLK_SPIFI, CLK_SPIFI, CLK_BASE_SPIFI},
	{CLK_MX_BUS, CLK_MX_QEI, CLK_BASE_MX},
#if 0
#if defined(CHIP_LPC43XX)
	{CLK_PERIPH_BUS, CLK_PERIPH_SGPIO, CLK_BASE_PERIPH},
#endif
	{CLK_USB0, CLK_USB0, CLK_BASE_USB0},
	{CLK_USB1, CLK_USB1, CLK_BASE_USB1},
#if defined(CHIP_LPC43XX)
	{CLK_SPI, CLK_SPI, CLK_BASE_SPI},
	{CLK_VADC, CLK_VADC, CLK_BASE_VADC},
#endif
	{CLK_APLL, CLK_APLL, CLK_BASE_APLL},
	{CLK_APB2_UART3, CLK_APB2_UART3, CLK_BASE_UART3},
	{CLK_APB2_UART2, CLK_APB2_UART2, CLK_BASE_UART2},
	{CLK_APB2_UART1, CLK_APB2_UART1, CLK_BASE_UART1},
	{CLK_APB2_UART0, CLK_APB2_UART0, CLK_BASE_UART0},
	{CLK_APB2_SSP1, CLK_APB2_SSP1, CLK_BASE_SSP1},
	{CLK_APB2_SSP0, CLK_APB2_SSP0, CLK_BASE_SSP0},
	{CLK_APB2_SDIO, CLK_APB2_SDIO, CLK_BASE_SDIO},
	{CLK_CCU2_LAST, CLK_CCU2_LAST, CLK_BASE_NONE}
#endif
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Test PLL input values for a specific frequency range */
static uint32_t Chip_Clock_TestMainPLLMultiplier(uint32_t InputHz, uint32_t TestMult, uint32_t MinHz, uint32_t MaxHz)
{
	uint32_t TestHz = TestMult * InputHz;

	if ((TestHz < MinHz) || (TestHz > MAX_CLOCK_FREQ) || (TestHz > MaxHz)) {
		TestHz = 0;
	}

	return TestHz;
}

/* Returns clock rate out of a divider */
static uint32_t Chip_Clock_GetDivRate(CGU_CLKIN_T clock, CGU_IDIV_T divider)
{
	CGU_CLKIN_T input;
	uint32_t div;

	input = Chip_Clock_GetDividerSource(divider);
	div = Chip_Clock_GetDividerDivisor(divider);
	return Chip_Clock_GetClockInputHz(input) / (div + 1);
}

/* Finds the base clock for the peripheral clock */
static CGU_BASE_CLK_T Chip_Clock_FindBseClock(CCU_CLK_T clk)
{
	CGU_BASE_CLK_T baseclk = CLK_BASE_NONE;
	int i = 0;

	while ((baseclk == CLK_BASE_NONE) && (periph_to_base[i].clkbase != baseclk)) {
		if ((clk >= periph_to_base[i].clkstart) && (clk <= periph_to_base[i].clkend)) {
			baseclk = periph_to_base[i].clkbase;
		}
		else {
			i++;
		}
	}

	return baseclk;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Enables the crystal oscillator */
void Chip_Clock_EnableCrystal(void)
{
	uint32_t OldCrystalConfig = LPC_CGU->XTAL_OSC_CTRL;

	/* Clear bypass mode */
	OldCrystalConfig &= (~2);
	if (OldCrystalConfig != LPC_CGU->XTAL_OSC_CTRL) {
		LPC_CGU->XTAL_OSC_CTRL = OldCrystalConfig;
	}

	/* Enable crystal oscillator */
	OldCrystalConfig &= (~1);
	if (CRYSTAL_MAIN_FREQ_IN >= 20000000) {
		OldCrystalConfig |= 4;	/* Set high frequency mode */

	}
	LPC_CGU->XTAL_OSC_CTRL = OldCrystalConfig;
}

/* Disables the crystal oscillator */
void IP_Clock_DisableCrystal(void)
{
	/* Disable crystal oscillator */
	LPC_CGU->XTAL_OSC_CTRL &= (~1);
}

/* Configures the main PLL */
uint32_t Chip_Clock_SetupMainPLLHz(CGU_CLKIN_T Input, uint32_t MinHz, uint32_t DesiredHz, uint32_t MaxHz)
{
	uint32_t freqin = Chip_Clock_GetClockInputHz(Input);
	uint32_t Mult, LastMult, MultEnd;
	uint32_t freqout, freqout2;

	if (DesiredHz != 0xFFFFFFFF) {
		/* Test DesiredHz rounded down */
		Mult = DesiredHz / freqin;
		freqout = Chip_Clock_TestMainPLLMultiplier(freqin, Mult, MinHz, MaxHz);

		/* Test DesiredHz rounded up */
		Mult++;
		freqout2 = Chip_Clock_TestMainPLLMultiplier(freqin, Mult, MinHz, MaxHz);

		if (freqout && !freqout2) {	/* rounding up is no good? set first multiplier */
			Mult--;
			return Chip_Clock_SetupMainPLLMult(Input, Mult);
		}
		if (!freqout && freqout2) {	/* didn't work until rounded up? set 2nd multiplier */
			return Chip_Clock_SetupMainPLLMult(Input, Mult);
		}

		if (freqout && freqout2) {	/* either multiplier okay? choose closer one */
			if ((DesiredHz - freqout) > (freqout2 - DesiredHz)) {
				Mult--;
				return Chip_Clock_SetupMainPLLMult(Input, Mult);
			}
			else {
				return Chip_Clock_SetupMainPLLMult(Input, Mult);
			}
		}
	}

	/* Neither multiplier okay? Try to start at MinHz and increment.
	   This should find the highest multiplier that is still good */
	Mult = MinHz / freqin;
	MultEnd = MaxHz / freqin;
	LastMult = 0;
	while (1) {
		freqout = Chip_Clock_TestMainPLLMultiplier(freqin, Mult, MinHz, MaxHz);

		if (freqout) {
			LastMult = Mult;
		}

		if (Mult >= MultEnd) {
			break;
		}
		Mult++;
	}

	if (LastMult) {
		return Chip_Clock_SetupMainPLLMult(Input, LastMult);
	}

	return 0;
}

/* Directly set the PLL multipler */
uint32_t Chip_Clock_SetupMainPLLMult(CGU_CLKIN_T Input, uint32_t mult)
{
	uint32_t freq = Chip_Clock_GetClockInputHz(Input);
	uint32_t msel = 0, nsel = 0, psel = 0, pval = 1;
	uint32_t PLLReg = LPC_CGU->PLL1_CTRL;

	freq *= mult;
	msel = mult - 1;

	PLLReg &= ~(0x1F << 24);/* clear input source bits */
	PLLReg |= Input << 24;	/* set input source bits to parameter */

	/* Clear other PLL input bits */
	PLLReg &= ~((1 << 6) |	/* FBSEL */
				(1 << 1) |	/* BYPASS */
				(1 << 7) |	/* DIRECT */
				(0x03 << 8) | (0xFF << 16) | (0x03 << 12));	/* PSEL, MSEL, NSEL- divider ratios */

	if (freq < 156000000) {
		/* psel is encoded such that 0=1, 1=2, 2=4, 3=8 */
		while ((2 * (pval) * freq) < 156000000) {
			psel++;
			pval *= 2;
		}

		PLLReg |= (msel << 16) | (nsel << 12) | (psel << 8) | (1 << 6);	/* dividers + FBSEL */
	}
	else if (freq < 320000000) {
		PLLReg |= (msel << 16) | (nsel << 12) | (psel << 8) | (1 << 7) | (1 << 6);	/* dividers + DIRECT + FBSEL */
	}
	else {
		Chip_Clock_DisableMainPLL();
		return 0;
	}
	LPC_CGU->PLL1_CTRL = PLLReg & ~(1 << 0);

	return freq;
}

/* Returns the frequency of the main PLL */
uint32_t Chip_Clock_GetMainPLLHz(void)
{
	uint32_t PLLReg = LPC_CGU->PLL1_CTRL;
	uint32_t freq = Chip_Clock_GetClockInputHz((CGU_CLKIN_T) ((PLLReg >> 24) & 0xF));
	uint32_t msel, nsel, psel, direct, fbsel;
	uint32_t m, n, p;
	const uint8_t ptab[] = {1, 2, 4, 8};

	/* No lock? */
	if (!(LPC_CGU->PLL1_STAT & 1)) {
		return 0;
	}

	msel = (PLLReg >> 16) & 0xFF;
	nsel = (PLLReg >> 12) & 0x3;
	psel = (PLLReg >> 8) & 0x3;
	direct = (PLLReg >> 7) & 0x1;
	fbsel = (PLLReg >> 6) & 0x1;

	m = msel + 1;
	n = nsel + 1;
	p = ptab[psel];

	if (direct || fbsel) {
		return m * (freq / n);
	}

	return (m / (2 * p)) * (freq / n);
}

/* Disables the main PLL */
void Chip_Clock_DisableMainPLL(void)
{
	/* power down main PLL */
	LPC_CGU->PLL1_CTRL |= 1;
}

/* Returns the lock status of the main PLL */
bool Chip_Clock_MainPLLLocked(void)
{
	/* Return true if locked */
	return (bool) (LPC_CGU->PLL1_STAT & 1);
}

/* Sets up a CGU clock divider and it's input clock */
void Chip_Clock_SetDivider(CGU_IDIV_T Divider, CGU_CLKIN_T Input, uint32_t Divisor)
{
	uint32_t reg = LPC_CGU->IDIV_CTRL[Divider];

	Divisor--;

	if (Input != CLKINPUT_PD) {
		/* Mask off bits that need to changes */
		reg &= ~((0x1F << 24) | 1 | (0xF << 2));

		/* Enable autoblocking, clear PD, and set clock source & divisor */
		LPC_CGU->IDIV_CTRL[Divider] = reg | (1 << 11) | (Input << 24) | (Divisor << 2);
	}
	else {
		LPC_CGU->IDIV_CTRL[Divider] = reg | 1;	/* Power down this divider */
	}
}

/* Gets a CGU clock divider source */
CGU_CLKIN_T Chip_Clock_GetDividerSource(CGU_IDIV_T Divider)
{
	uint32_t reg = LPC_CGU->IDIV_CTRL[Divider];

	if (reg & 1) {	/* divider is powered down */
		return CLKINPUT_PD;
	}

	return (CGU_CLKIN_T) ((reg >> 24) & 0x1F);
}

/* Gets a CGU clock divider divisor */
uint32_t Chip_Clock_GetDividerDivisor(CGU_IDIV_T Divider)
{
	return (CGU_CLKIN_T) ((LPC_CGU->IDIV_CTRL[Divider] >> 2) & 0xF);
}

/* Returns the frequency of the specified input clock source */
uint32_t Chip_Clock_GetClockInputHz(CGU_CLKIN_T input)
{
	uint32_t rate = 0;

	switch (input) {
	case CLKIN_32K:
		rate = CRYSTAL_32K_FREQ_IN;
		break;

	case CLKIN_IRC:
		rate = CGU_IRC_FREQ;
		break;

	case CLKIN_ENET_RX:
#if defined(USE_RMII)
		/* In RMII mode, this clock is not attached */
#else
		/* MII mode requires 25MHz clock */
		rate = 25000000;
#endif
		break;

	case CLKIN_ENET_TX:
#if defined(USE_RMII)
		/* MII mode requires 50MHz clock */
		rate = 50000000;
#else
		/* MII mode requires 25MHz clock */
		rate = 25000000;
#endif
		break;

	case CLKIN_CLKIN:
#if defined(EXTERNAL_CLKIN_FREQ_IN)
		rate = EXTERNAL_CLKIN_FREQ_IN;
#else
		/* Assume no clock in if a rate wasn't defined */
#endif
		break;

	case CLKIN_CRYSTAL:
		rate = CRYSTAL_MAIN_FREQ_IN;
		break;

	case CLKIN_USBPLL:
		rate = 0; // FIXME
		break;

	case CLKIN_AUDIOPLL:
		rate = 0; // FIXME
		break;

	case CLKIN_MAINPLL:
		rate = Chip_Clock_GetMainPLLHz();
		break;

	case CLKIN_IDIVA:
		rate = Chip_Clock_GetDivRate(input, CLK_IDIV_A);
		break;

	case CLKIN_IDIVB:
		rate = Chip_Clock_GetDivRate(input, CLK_IDIV_B);
		break;

	case CLKIN_IDIVC:
		rate = Chip_Clock_GetDivRate(input, CLK_IDIV_C);
		break;

	case CLKIN_IDIVD:
		rate = Chip_Clock_GetDivRate(input, CLK_IDIV_D);
		break;

	case CLKIN_IDIVE:
		rate = Chip_Clock_GetDivRate(input, CLK_IDIV_E);
		break;

	case CLKINPUT_PD:
		rate = 0;
		break;

	default:
		break;
	}

	return rate;
}

/* Returns the frequency of the specified base clock source */
uint32_t Chip_Clock_GetBaseClocktHz(CGU_BASE_CLK_T clock)
{
	return Chip_Clock_GetClockInputHz(Chip_Clock_GetBaseClock(clock));
}

/* Sets a CGU Base Clock clock source */
void Chip_Clock_SetBaseClock(CGU_BASE_CLK_T BaseClock, CGU_CLKIN_T Input, bool autoblocken, bool powerdn)
{
	uint32_t reg = LPC_CGU->BASE_CLK[BaseClock];

	if (BaseClock < CLK_BASE_NONE) {
		if (Input != CLKINPUT_PD) {
			/* Mask off fields we plan to update */
			reg &= ~((0x1F << 24) | 1 | (1 << 11));

			if (autoblocken) {
				reg |= (1 << 11);
			}
			if (powerdn) {
				reg |= (1 << 0);
			}

			/* Set clock source */
			reg |= (Input << 24);

			LPC_CGU->BASE_CLK[BaseClock] = reg;
		}
	}
	else {
		LPC_CGU->BASE_CLK[BaseClock] = reg | 1;	/* Power down this base clock */
	}
}

/*Enables a base clock source */
void Chip_Clock_EnableBaseClock(CGU_BASE_CLK_T BaseClock)
{
	if (BaseClock < CLK_BASE_NONE) {
		LPC_CGU->BASE_CLK[BaseClock] &= ~1;
	}
}

/* Disables a base clock source */
void Chip_Clock_DisableBaseClock(CGU_BASE_CLK_T BaseClock)
{
	if (BaseClock < CLK_BASE_NONE) {
		LPC_CGU->BASE_CLK[BaseClock] |= 1;
	}
}

/* Gets a CGU Base Clock clock source */
CGU_CLKIN_T Chip_Clock_GetBaseClock(CGU_BASE_CLK_T BaseClock)
{
	uint32_t reg = LPC_CGU->BASE_CLK[BaseClock];

	if (BaseClock >= CLK_BASE_NONE) {
		return CLKINPUT_PD;
	}

	/* base clock is powered down? */
	if (reg & 1) {
		return CLKINPUT_PD;
	}

	return (CGU_CLKIN_T) ((reg >> 24) & 0x1F);
}

/* Enables a peripheral clock and sets clock states */
void Chip_Clock_EnableOpts(CCU_CLK_T clk, bool autoen, bool wakeupen, int div)
{
	uint32_t reg = 1;

	if (autoen) {
		reg |= (1 << 1);
	}
	if (wakeupen) {
		reg |= (1 << 2);
	}

	/* Not all clocks support a divider, but we won't check that here. Only
	   dividers of 1 and 2 are allowed. Assume 1 if not 2 */
	if (div == 2) {
		reg |= (1 << 5);
	}

	/* Setup peripheral clock and start running */
	if (clk >= CLK_CCU2_START) {
		LPC_CCU2->CLKCCU[clk - CLK_CCU2_START].CFG = reg;
	}
	else {
		LPC_CCU1->CLKCCU[clk].CFG = reg;
	}
}

/* Enables a peripheral clock */
void Chip_Clock_Enable(CCU_CLK_T clk)
{
	/* Start peripheral clock running */
	if (clk >= CLK_CCU2_START) {
		LPC_CCU2->CLKCCU[clk - CLK_CCU2_START].CFG |= 1;
	}
	else {
		LPC_CCU1->CLKCCU[clk].CFG |= 1;
	}
}

/* Disables a peripheral clock */
void Chip_Clock_Disable(CCU_CLK_T clk)
{
	/* Stop peripheral clock */
	if (clk >= CLK_CCU2_START) {
		LPC_CCU2->CLKCCU[clk - CLK_CCU2_START].CFG &= ~1;
	}
	else {
		LPC_CCU1->CLKCCU[clk].CFG &= ~1;
	}
}

/* Returns a peripheral clock rate */
uint32_t Chip_Clock_GetRate(CCU_CLK_T clk)
{
	CGU_BASE_CLK_T baseclk;
	uint32_t reg, div, rate;

	/* Get CCU config register for clock */
	if (clk >= CLK_CCU2_START) {
		reg = LPC_CCU2->CLKCCU[clk - CLK_CCU2_START].CFG;
	}
	else {
		reg = LPC_CCU1->CLKCCU[clk].CFG;
	}

	/* Is the clock enabled? */
	if (reg & 1) {
		/* Get base clock for this peripheral clock */
		baseclk = Chip_Clock_FindBseClock(clk);

		/* Get base clock rate */
		rate = Chip_Clock_GetBaseClocktHz(baseclk);

		/* Get divider for this clock */
		if (((reg >> 5) & 0x7) == 0) {
			div = 1;
		}
		else {
			div = 2;/* No other dividers supported */

		}
		rate = rate / div;
	}
	else {
		rate = 0;
	}

	return rate;
}
