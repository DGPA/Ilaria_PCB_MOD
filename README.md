PROGETTO SEGANTINI SMART PARK

FIRMWARE ORIGINALE by LUCA CROTTI @ 2019

VERSIONI MOD by THE ILARIA TEAM (Norman Mulinacci, Romeo Lorenzo e Diego Porras) @ 2020

      HW: 
      - PCB: ILARIA_PCBV4
      - MONITOR OLED 1.3" 128x64 SH1106 interfaccia I2C
      - ESP32 ESP32-WROVER-B
      
                         ____________________
                        |                     |
               3V3--- o-|-3V3             GND-|-o
                      o-|-EN              D23-|-o --- MOSI SDCARD
      MICS4514-COx -- o-|-VP(D36)         D22-|-o --- SCL OLED+BME+MICS6814
      MICS4514-NOx -- o-|-VN(D39)     (D1)TX0-|-o
      MICS4514-pre -- o-|-D34         (D3)RX0-|-o
    MQ-7(COx)A0 ----- o-|-D35             D21-|-o --- SDA OLED+BME+MICS6814
    MQ-7(COx)D0 ----- o-|-D32             GND-|-o
          dip-1 ----- o-|-D33             D19-|-o --- MISO SDCARD u8
          dip-2 ----- o-|-D25             D18-|-o --- SCK SDCARD
          dip-3 ----- o-|-D26             D5 -|-o --- CS SDCARD
          dip-4 ----- o-|-D27             D17-|-o  -x-x- non usare! protetto
        RX SENS PM--- o-|-D14             D16-|-o  -x-x- non usare! protetto
        TX SENS PM--- o-|-D12             D4 -|-o --- PIN ATTIVAZ. MOSFET
                      o-|-GND             D0 -|-o
                      o-|-D13             D2 -|-o
                      o-|-SD2  ---xxx     D15-|-o  -x-x- non usare! protetto
                      o-|-SD3  ---xxx---  SD1-|-o  -x-x- non usare! protetto
                      o-|-CMD  ---xxx---  SD0-|-o  -x-x- non usare! protetto
       VCC-SD+OLED--- o-|-5Vin    xxx---  CLK-|-o  -x-x- non usare! protetto
                        |                     |
                        |      _______        |
                        |     |       |       |
                        |     |       |       |
                        |_____|       |_______|
                              |_______|
                            
	
Changelog:

VERSIONE 25d:

	- risolto problema di memory leak nel modo di calcolare la media delle misurazioni
	- aggiunto spegnimento del sensore Multichannel durante lo sleep
	- sistemato piccolo errore nel log del file su SD
	- sistemato allineamento del log su seriale
	- rimosso il sistema di gestione per fasi, ora è più semplice lavorare coi blocchi di codice
	- rimossa chiamata a libreria non utilizzata, commentate parti di codice inutilizzate
	- cambiate alcune modalità di gestione dello schermo integrato
	- aggiornati i crediti nella schermata iniziale

VERSIONE 25c2:

	- aggiunta possibilità di inviare solo la media di (x) misurazioni, effettuate ogni (x) secondi
	- aggiunti i valori precedentemente rimossi dal log su csv

VERSIONE 25b:

	- trascritta e modificata funzione che calcola data per compatibilità con nuove versioni di NTPClient
	- cambiata modalità di log su SD (da .txt a .csv), cambiato logging con valori in virgola
	- aggiunta schermata con ulteriori valori dal Multichannel Gas Sensor
	- modificato comportamento in caso di errore su file RETE.txt
	- unificato aspetto delle schermate del sistema e cambiata icona della scheda microsd

VERSIONE 24:

	- aggiunto delay di preriscaldamento dei sensori all'avvio
	- modificata modalità di LOG su seriale e scheda SD (per maggiore conformità col CSV)
	- aggiunto invio dati del Multichannel Gas Sensor al server
	- aggiunta distinzione tra centralina fissa o mobile (tramite variabile booleana)
	- risolto bug di misura COppm con MQ-7 (corretto pin35 e variabile di upload su server)


Versioni antecedenti (Luca Crotti):


VERSIONE 23:

	- test con diverse librerie, ottimizzato l'ordine delle chiamate
	- corretto richiamo libreria NTPClient (serve libreria specifica, modificata in riga 48)

VERSIONE 22:

	- aggiunta del sensore MICS-6814 grove
	- il segnale del CO è relativo ora a questo nuovo sensore.
	- aggiunta schermata 2 dove compaiono anche gli altri valori letti dal sensore

VERSIONE 21:

	- aggiunta schermata di MAC ADDRESS alla fine del LOOP
	- spedizione su nuovo sito Segantini (accordi con Carlo per la parte di protocollo)
