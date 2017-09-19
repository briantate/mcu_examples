/*
 * C21_Plum_Triac_Dimmer_BareMetal.c
 *
 * Created: 3/21/2016 9:12:44 PM
 * Author : brian.tate
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */ 


#include "sam.h"
#define F_CLK 1000000 

#define CC0_VAL 0x0FFF
#define CC1_VAL CC0_VAL/2

/*  Prototypes */
void delay(uint32_t ms);
void init_led(void);
void init_TC0(void);
void init_ext_interrupt(void);
void init_events(void);


int main(void)
{
	init_led();
	init_TC0();
	init_ext_interrupt();
	init_events();
	
	//enable global interrupts
	//__DMB();
	//__enable_irq();


	while(1) {
/*
		//LED toggle
		PORT->Group[0].OUTTGL.reg |= PORT_PA15;
		delay(500);
*/

	}
}

void delay(uint32_t ms){
	uint32_t volatile i;
	for(i=0; i< F_CLK * ms/1000; i++){
		//do nothing
	}
}

void init_led(void){
	PORT->Group[0].DIRSET.reg |= PORT_PA15;
	PORT->Group[0].OUTSET.reg |= PORT_PA15;
}

void init_TC0(void){

	//TC0 PORTs 
	//Waveform output on WO[1] (PB13)
	PORT->Group[1].DIRSET.reg |= (1U<<13U);
	PORT->Group[1].PMUX[6].reg |= PORT_PMUX_PMUXO(0x4); //Set port MUX pin 13 for peripheral function E (0x4) - TC0/WO[1]
	PORT->Group[1].PINCFG[13].reg |= PORT_PINCFG_PMUXEN; //Enable the PMUX
	//Single pulse on overflow on WO[0] (PB12)
	PORT->Group[1].DIRSET.reg |= (1U<<12U);
	PORT->Group[1].PMUX[6].reg |= PORT_PMUX_PMUXE(0x4); //port MUX pin 12 for peripheral function E (0x4) - TC0/WO[0]
	PORT->Group[1].PINCFG[12].reg |= PORT_PINCFG_PMUXEN; //Enable the PMUX


	//TC0 clocks 
	MCLK->APBCMASK.bit.TC0_ = 1;   //Enable TC bus clock CLK_TC0_APB
	GCLK->PCHCTRL[30].reg |= GCLK_PCHCTRL_GEN_GCLK0 | 
	                         GCLK_PCHCTRL_CHEN;

	//TC0 registers
	//counter mode default -- CTRLA.MODE = COUNT16
	TC0->COUNT16.CC[0].reg = CC0_VAL;
	TC0->COUNT16.CC[1].reg = CC1_VAL;
	TC0->COUNT16.WAVE.reg =  TC_WAVE_WAVEGEN_MPWM; //Match PWM - Count to CC0 - toggle on CC0 and CC1
	TC0->COUNT16.CTRLBSET.bit.ONESHOT = 1;
	TC0->COUNT16.DRVCTRL.reg = TC_DRVCTRL_INVEN0 |      //invert WO[0]
	                           TC_DRVCTRL_INVEN1;       //invert WO[1]
	TC0->COUNT16.EVCTRL.reg |= TC_EVCTRL_EVACT_START |  //set event action to start timer
	                           TC_EVCTRL_TCEI;          //enable incoming events
	TC0->COUNT16.CTRLA.bit.RUNSTDBY = 1; //Enable sleep mode operation
	TC0->COUNT16.CTRLA.bit.ENABLE = 1;//Enable CTRLA.ENABLE = 1
}

void init_ext_interrupt(void){
	/*a pulse on external pin PB14 should generate an event which is used
      by timer, but it should not disturb the CPU*/
	
	//Port PB14 is input
	PORT->Group[1].PMUX[7].reg    |= PORT_PMUX_PMUXE(0x0);  //Set port MUX pin 14 for peripheral function A (0x0) Ext. Int.
	PORT->Group[1].PINCFG[14].reg |= PORT_PINCFG_INEN |     //enable input
	                                 PORT_PINCFG_PULLEN |   //enable pullup
									 PORT_PINCFG_PMUXEN;    //set the mux to use peripheral functions

	//EIC clocks
	MCLK->APBAMASK.bit.EIC_ = 1; //enable CLK_EIC_APB
	GCLK->PCHCTRL[2].reg |= GCLK_PCHCTRL_GEN_GCLK0 |
	                        GCLK_PCHCTRL_CHEN;

	//EIC registers
	//EIC->INTENSET.reg |= (1<<14);//enable interrupts INTENSET -- debug only, we don't want an interrupt
	EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE6_RISE | 
	                      EIC_CONFIG_FILTEN6;
	EIC->ASYNCH.reg |= (1<<14); //Configure Asynchronous edge detection for operation in all sleep modes ASYNCH.ASYNCH[x]
	EIC->EVCTRL.reg |= (1<<14);
	EIC->CTRLA.bit.ENABLE = 1;

}

void init_events(void){

	//EVSYS clocsk
	MCLK->APBCMASK.bit.EVSYS_ = 1;                                 //enable clock CLK_EVSYS_APB
	GCLK->PCHCTRL[6].reg   |= GCLK_PCHCTRL_GEN_GCLK0 | 
							  GCLK_PCHCTRL_CHEN;                   //enable GCLK_EVSYS_CHANNEL_0

	//EVSYS registers
	//Event users must be configured before channel and Generators
	EVSYS->USER[23].reg   |=    EVSYS_USER_CHANNEL(0x01);          //configure event user USER[23] (TC0) to use channel 0
	EVSYS->CHANNEL[0].reg |=	EVSYS_CHANNEL_EVGEN(0x1c) |        //configure event generator CHANNELn.EVGEN for ext int 14
								EVSYS_CHANNEL_PATH_ASYNCHRONOUS |  //configure channel path asynch/sync/resynch CHANNELn.PATH
								//EVSYS_CHANNEL_EDGSEL_RISING_EDGE | //these bits must be 0 when using asynch mode
								EVSYS_CHANNEL_RUNSTDBY;            //configure to run in standby mode CHANNELn.RUNSTDBY
}

/*//for debug purposes only -- In this case, we want an event, not an interrupt
void EIC_Handler(void){ 
	//do something
	if ( EIC->INTFLAG.reg & (1ul << 14)){//check for ext. int. 14
		PORT->Group[0].OUTTGL.reg |= PORT_PA15;
		EIC->INTFLAG.reg |= (1ul << 14);//clear ext int 14
	}
}
*/
