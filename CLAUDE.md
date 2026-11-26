# Progetto: nuovo FB posizionatore motore (AxisAS_Mng)

## Obiettivo
Riscrivere da zero il function block usato in azienda per controllare la posizione di un motore
e portarlo in una posizione richiesta. L'FB attuale è misto KOP/SCL ed è esportato con VCI in
questa cartella (RepoPosizionatore).

## Vincoli
- Tutto in SCL.
- Compatibile S7-1200 e S7-1500.
- Metodo di lavoro: si modificano i file nel workspace VCI, poi si sincronizzano nel progetto TIA
  per compilare e testare (PLCSIM Advanced). Claude non vede TIA Portal: gli errori di
  compilazione vengono incollati in chat.

## Funzioni da mantenere
- Ricerca livello
- Autocalcolo velocità / accelerazione
- Anello aperto
- Sincro master/slave: per ora mantenere le strutture esistenti; in seguito diventerà un FB
  separato che raccorda le posizioni di 2 motori (es. mettifalda con asse verticale + orizzontale)

## Profilo di moto
- Profilo trapezoidale con limitazione del jerk (sostituisce le vecchie curve log/cos).
- Di base SENZA inseguimento di un asse virtuale (usato anche su meccaniche con forte inerzia).
- Inseguimento dell'asse virtuale disponibile come opzione da configurazione.

## Strumenti di contorno
- Interfaccia comando / parametri / feedback in WinCC Advanced, con grafico della traiettoria
  e della posizione reale.
- Simulatore su PC che simula solo l'encoder.

## Regole di lavoro
- Non modificare i file dell'FB originale: il nuovo FB va in file nuovi.
- Prima di cambiamenti grossi, proporre il piano e aspettare conferma.
- Tenere aggiornata la sezione "Stato" qui sotto a fine sessione.

## Stato
Aggiornato 26/11/2026. Questo file è nella radice del repo (prima era sul Desktop, ricreato il 26/11).
- FB originale (non toccare): `Axis_As/Blocchi di programma/Axis_As/AxisAS_Mng_v2.0.xml` (+ v1.3 e base),
  UDT `Tipi di dati PLC/AxisAS_2.0/*`. La struttura Sincro v2.0 è solo dichiarata, mai usata nel codice.
- Nuovo FB in `Axis_As/Blocchi di programma/Axis_As_v4/` (SCL), deciso di proseguire su questa bozza:
  - `AxisAS_Mng_v4.scl` (v0.4): macchina a stati (0/10/20/30/40/50/60/70/71/99), interfaccia IN/OUT piatta,
    Cfg come IN_OUT (`AxisAS_CFG_Type_v4`), `Cfg.xPosLoop` = FALSE anello aperto (default) / TRUE inseguimento virtuale.
  - `AxisAS_Profile_v4.scl` (FB trapezio con jerk), `AxisAS_RampDist_v4.scl` (FC spazio di frenata).
  - Contorno: `AxisAS_Test_v4.scl` (banco di prova, Cmd da HMI), `AxisAS_Trace_v4.scl` (traiettoria prevista/reale
    per trend WinCC, v0.2: 999 punti = massimo di una curva del trend view, `N_LAST` = 998 = limite degli array,
    passo = durata prevista x 1,5 / 998; registra ancora `POST_TIME` = 0,5 s dopo l'arrivo, poi velocità reale 0 nei
    punti restanti), DB `AxisAS_SimIO` (DB900, accesso standard, offset fissi).
- v0.3 (24/11): ingressi per il futuro FB Sincro `ifSpeedOverride`, `ixHold`, `ixSincroError` (allarme 5), uscita `oxHold`;
  `Cfg.fKp_PosDec` (Kp a piramide come v2.0); `ifTimeScan` opzionale (0 = RUNTIME); validazione parametri anche su JOG
  e soglie autospeed; tabella corrispondenze v2.0 -> v4 nell'header dell'FB. Non ancora compilata in TIA.
- Simulatore PC: `Simulatore/AxisAS_Sim_v4.py` (+ `AxisAS_Sim_v4.json` parametri salvati, IP 192.168.0.1),
  Python + python-snap7 + tkinter, modello motore asincrono/inverter/meccanica/freno, sensore livello virtuale e
  guasti simulabili. Offset verificati con AxisAS_SimIO. Avvio: `python Simulatore\AxisAS_Sim_v4.py`.
  Copiato il 24/11 da `RepoPosizionatore\POSIZIONATORE_File\Pos Simulator\` (lì restano anche `PosSim.py`,
  vecchio simulatore di giugno 2026, e `_to_delete\AxisAS_Sim_v4_rev3.py`, copia precedente): la versione di
  riferimento è quella nel repo.
  - 26/11: collegamento robusto: scollega solo dopo 1 s di errori continui (prima ~55 ms), ricollegamento automatico
    ogni 2 s in un thread separato, I/O snap7 fuori dal lock della simulazione, cadute/ricollegamenti/ultimo errore
    visibili sotto la barra in alto, ciclo medio e massimo. Provato offline con PLC finto, NON ancora con PLCSIM Advanced.
- Prove su PLCSIM (26/11), movimento 0 -> 200 mm, 100 mm/s, 500 mm/s^2:
  - Il trend WinCC mostrava solo 200 punti dei 1000 di Trace (~0,68 s di ~2,3 s). Trace portato a 999 punti
    (26/11): nel trend view impostare 999 valori per curva e adeguare i tag array HMI (Ax4_Trace_aPlanPos ecc.).
  - Movimento 0 -> 500 mm con 999 punti: posizione prevista e reale coincidono. Velocità reale con denti 91..106 mm/s:
    CONFERMATO artefatto di misura (modello Python dello stimatore: encoder scritto ogni 5,3 ms, letto da OB a 10 ms
    in modo asincrono -> 91,4..105,8 con velocità vera costante 100). Simulatore portato a SIM_PERIOD = 2 ms
    (atteso 96..103; storico 60000 campioni). Velocità reale rimasta a 4,1 mm/s dopo l'arrivo: corretto in Trace v0.2.
  - Movimento 0 -> 200 mm con sim a 2 ms: denti 95..104 (ok). Il motore frena in ritardo (tAdvStop = 0) e arriva in
    quota a ~14 mm/s senza fase a fMinSpeed: provare tAdvStop 40..60 ms guardando ofOvershoot. Gradino piatto della
    velocità stimata dopo l'arresto (tenuta 100 ms di ENC_STILL_T): corretto in Mng_v4 v0.4 (limite fisico a encoder
    fermo; modello Python: "fermo" riconosciuto dopo ~20 ms invece di ~120 ms, regime invariato). Non ancora compilata.
- Da fare:
  - compilare v0.4 in TIA e provare in PLCSIM Advanced;
  - verificare RUNTIME/LReal sulla S7-1200 di destinazione;
  - adeguare Trace_v4 a override e hold (proposta: nuova uscita `ofSpeedCruise` con override applicato e
    ricalcolo della traiettoria alla ripartenza dopo una pausa; il ricalcolo a override variabile insieme all'FB Sincro);
  - provare il simulatore modificato con PLCSIM e capire la causa delle cadute (leggere l'errore mostrato);
  - git non è installato/nel PATH: installarlo e fare il primo commit;
  - poi FB Sincro separato (es. mettifalda verticale + orizzontale).
