ILARIA_PCB_WROVER_MOD firmware changelog

Firmware originale: Luca Crotti

Versione MOD di Norman Mulinacci, Romeo Lorenzo E Diego Porras (2020)


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
