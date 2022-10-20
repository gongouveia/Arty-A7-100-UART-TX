----------------------------------------------------------------------------
--    GPIO_Demo.vhd -- ARTY A7 /UART 
----------------------------------------------------------------------------
-- Author: Gonçalo Gouveia
-- onControl technologies
----------------------------------------------------------------------------
--
--    All UART communication can be captured by attaching the UART port to a
-- computer running a Terminal program with 9600 Baud Rate, 8 data bits, no
-- parity, and 1 stop bit.                                                                
----------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

--The IEEE.std_logic_unsigned contains definitions that allow
--std_logic_vector types to be used with the + operator to instantiate a
--counter.
use IEEE.std_logic_unsigned.all;

entity GPIO_demo is
    Port (
           BTN             : in  STD_LOGIC_VECTOR (3 downto 0);
           CLK             : in  STD_LOGIC;
           UART_TXD     : out  STD_LOGIC

              );
end GPIO_demo;

architecture Behavioral of GPIO_demo is



component UART_TX_CTRL
Port(
    SEND : in std_logic;
    DATA : in std_logic_vector(7 downto 0);
    CLK : in std_logic;          
    READY : out std_logic;
    UART_TX : out std_logic
    );
end component;

component debouncer
Generic(
        DEBNC_CLOCKS : integer;
        PORT_WIDTH : integer);
Port(
        SIGNAL_I : in std_logic_vector(3 downto 0);
        CLK_I : in std_logic;          
        SIGNAL_O : out std_logic_vector(3 downto 0)
        );
end component;




--The type definition for the UART state machine type. Here is a description of what
--occurs during each state:
-- RST_REG     -- Do Nothing. This state is entered after configuration or a user reset.
--                The state is set to LD_INIT_STR.
-- LD_INIT_STR -- The Welcome String is loaded into the sendStr variable and the strIndex
--                variable is set to zero. The welcome string length is stored in the StrEnd
--                variable. The state is set to SEND_CHAR.
-- SEND_CHAR   -- uartSend is set high for a single clock cycle, signaling the character
--                data at sendStr(strIndex) to be registered by the UART_TX_CTRL at the next
--                cycle. Also, strIndex is incremented (behaves as if it were post
--                incremented after reading the sendStr data). The state is set to RDY_LOW.
-- RDY_LOW     -- Do nothing. Wait for the READY signal from the UART_TX_CTRL to go low,
--                indicating a send operation has begun. State is set to WAIT_RDY.
-- WAIT_RDY    -- Do nothing. Wait for the READY signal from the UART_TX_CTRL to go high,
--                indicating a send operation has finished. If READY is high and strEnd =
--                StrIndex then state is set to WAIT_BTN, else if READY is high and strEnd /=
--                StrIndex then state is set to SEND_CHAR.
-- WAIT_BTN    -- Do nothing. Wait for a button press on BTNU, BTNL, BTND, or BTNR. If a
--                button press is detected, set the state to LD_BTN_STR.
-- LD_BTN_STR  -- The Button String is loaded into the sendStr variable and the strIndex
--                variable is set to zero. The button string length is stored in the StrEnd
--                variable. The state is set to SEND_CHAR.
type UART_STATE_TYPE is (RST_REG, LD_INIT_STR, SEND_CHAR, RDY_LOW, WAIT_RDY, WAIT_BTN, LD_BTN_STR);




--The CHAR_ARRAY type is a variable length array of 8 bit std_logic_vectors.
--Each std_logic_vector contains an ASCII value and represents a character in
--a string. The character at index 0 is meant to represent the first
--character of the string, the character at index 1 is meant to represent the
--second character of the string, and so on.
type CHAR_ARRAY is array (integer range<>) of std_logic_vector(7 downto 0);

constant TMR_CNTR_MAX : std_logic_vector(26 downto 0) := "101111101011110000100000000"; --100,000,000 = clk cycles per second
constant TMR_VAL_MAX : std_logic_vector(3 downto 0) := "1001"; --9

constant RESET_CNTR_MAX : std_logic_vector(17 downto 0) := "110000110101000000";-- 100,000,000 * 0.002 = 200,000 = clk cycles per 2 ms


constant WELCOME_STR : CHAR_ARRAY(0 to 16) :=                (X"0A",  --\n
                                                              X"0D",  --\r
                                                              X"41",  --A
                                                              X"52",  --R
                                                              X"54",  --T
                                                              X"59",  --Y
                                                              X"20",  --
                                                              X"55",  --U
                                                              X"41",  --A
                                                              X"52",  --R
                                                              X"54",  --T
                                                              X"20",  --
                                                              X"44",  --D
                                                              X"45",  --E
                                                              X"4D",  --M
                                                              X"4F",  --O
                                                              X"21",  --!
                                                              X"20",  --
                                                              X"20",  --
                                                              X"20",  --
                                                              X"0A",  --\n
                                                              X"0A",  --\n
                                                              X"0D"); --\r
                                                              
--Button press string definition.
constant BTN_STR : CHAR_ARRAY(0 to 8) :=                    (X"42",  --B
                                                              X"75",  --u
                                                              X"74",  --t
                                                              X"74",  --t
                                                              X"6F",  --o
                                                              X"6E",  --n
                                                              X"20",  --                                                           
                                                              X"0A",  --\n
                                                              X"0D"); --\r


constant MAX_STR_LEN : integer := WELCOME_STR'LENGTH;
constant WELCOME_STR_LEN : natural := WELCOME_STR'LENGTH;
constant BTN_STR_LEN : natural := BTN_STR'LENGTH;


--Contains the current string being sent over uart.
signal sendStr : CHAR_ARRAY(0 to (MAX_STR_LEN - 1));

--Contains the length of the current string being sent over uart.
signal strEnd : natural;

--Contains the index of the next character to be sent over uart
--within the sendStr variable.
signal strIndex : natural;

--Used to determine when a button press has occured
signal btnReg : std_logic_vector (3 downto 0) := "0000";
signal btnDetect : std_logic;

--UART_TX_CTRL control signals
signal uartRdy :  std_logic;
signal uartSend : std_logic := '0';
signal uartData : std_logic_vector (7 downto 0):= "00000000";
signal uartTX :   std_logic;

--Current uart state signal
signal uartState : UART_STATE_TYPE := RST_REG;


-------------------------------------------------------------------------------------------------
--Debounced btn signals used to prevent single button presses
--from being interpreted as multiple button presses.
signal btnDeBnc : std_logic_vector(3 downto 0);

signal clk_cntr_reg : std_logic_vector (4 downto 0) := (others=>'0');

--signal pwm_val_reg : std_logic := '0';

--this counter counts the amount of time paused in the UART reset state
signal reset_cntr : std_logic_vector (17 downto 0) := (others=>'0');


signal count : integer :=0;
signal clk_out : std_logic :='0';

 --clk generation.For 100 MHz clock this generates 1 Hz clock.
signal uart_clock : integer := 100000000;

begin

----------------------------------------------------------
------              UART Clock                  -------
----------------------------------------------------------



 --clk generation.For 100 MHz clock this generates 1 Hz clock.
process(clk)
begin
    if(rising_edge(clk)) then
        count <=count+1;
        if(count = uart_clock) then
            clk_out <= not clk_out;
            count <=0;
         else
            clk_out <= '0';
    end if;
end if;
end process;


----------------------------------------------------------
------              UART Control                   -------
----------------------------------------------------------
--Messages are sent on reset and when a button is pressed.

--This counter holds the UART state machine in reset for ~2 milliseconds. This
--will complete transmission of any byte that may have been initiated during
--FPGA configuration due to the UART_TX line being pulled low, preventing a
--frame shift error from occuring during the first message.
process(CLK)
begin
  if (rising_edge(CLK)) then
    if ((reset_cntr = RESET_CNTR_MAX) or (uartState /= RST_REG)) then
      reset_cntr <= (others=>'0');
    else
      reset_cntr <= reset_cntr + 1;
    end if;
  end if;
end process;

--Next Uart state logic (states described above)

next_uartState_process : process (CLK)
begin
    if (rising_edge(CLK)) then
            
            case uartState is
            when RST_REG =>
        if (reset_cntr = RESET_CNTR_MAX) then
          uartState <= LD_INIT_STR;
        end if;
            when LD_INIT_STR =>
                uartState <= SEND_CHAR;
            when SEND_CHAR =>
                uartState <= RDY_LOW;
            when RDY_LOW =>
                uartState <= WAIT_RDY;
            when WAIT_RDY =>
                if (uartRdy = '1') then
                    if (strEnd = strIndex) then
                        uartState <= WAIT_BTN;                              --------------MUDAR AQUI 
                    else
                        uartState <= SEND_CHAR;
                    end if;
                end if;
            when WAIT_BTN =>
                if (clk_out = '1') then
                    uartState <= LD_BTN_STR;
                end if;
            when LD_BTN_STR =>
                uartState <= SEND_CHAR;
            when others=> --should never be reached
                uartState <= RST_REG;
            end case;
        
    end if;
end process;

--Loads the sendStr and strEnd signals when a LD state is
--is reached.
string_load_process : process (CLK)
begin
    if (rising_edge(CLK)) then
        if (uartState = LD_INIT_STR) then                  --mensagem inicial
            sendStr <= WELCOME_STR;
            strEnd <= WELCOME_STR'LENGTH;
        elsif (uartState = LD_BTN_STR) then
            sendStr(0 to BTN_STR'LENGTH-1) <= BTN_STR;
            strEnd <= BTN_STR'LENGTH;
        end if;
    end if;
end process;

--Conrols the strIndex signal so that it contains the index
--of the next character that needs to be sent over uart
char_count_process : process (CLK)
begin
    if (rising_edge(CLK)) then
        if (uartState = LD_INIT_STR or uartState = LD_BTN_STR) then
            strIndex <= 0;
        elsif (uartState = SEND_CHAR) then
            strIndex <= strIndex + 1;
        end if;
    end if;
end process;

--Controls the UART_TX_CTRL signals
char_load_process : process (CLK)
begin
    if (rising_edge(CLK)) then
        if (uartState = SEND_CHAR) then
            uartSend <= '1';
            uartData <= sendStr(strIndex);
        else
            uartSend <= '0';
        end if;
    end if;
end process;

--Component used to send a byte of data over a UART line.
Inst_UART_TX_CTRL: UART_TX_CTRL port map(
        SEND => uartSend,
        DATA => uartData,
        CLK => CLK,
        READY => uartRdy,
        UART_TX => uartTX
    );

UART_TXD <= uartTX;


-- #####################################




end Behavioral;


