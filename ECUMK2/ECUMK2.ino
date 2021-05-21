  //Aepl-Duino_11_04_21
//***Decalage et choix de courbe par Sphone***

//Description dÃ©taillÃ©e sur  http://loutrel.org/aeduino.php
//Si connectÃ© en Bluetooth, affiche le regime, l'avance
//PossibilitÃ© de decalage et changement de courbe au vol
//Decalage de + ou - 1 Ã  6 degrÃ©s sur courbe en cours.
//Pour -1 Ã  -6 degrÃ¨s , entrer 11,12...16
//Basculement de courbe :Entrer 8 pour courbe b, 9 pour courbe c  et 7 pour retour Ã  la courbe a d'origine

//  Allumage electronique programmable - Arduino Nano et compatibles

//******************************************************************************
//**************  Seulement  6 lignes Ã  renseigner obligatoirement.****************
//**********Ce sont:  Na  Anga  Ncyl  AngleCapteur  CaptOn  Dwell******************

//___1___ Na[] rÃ©gime moteur en t/mn
//Na[] et Anga[] doivent obligatoirement dÃ©buter puis se terminer par  0, et  contenir des valeurs  entiÃ¨res >=1
//Le dernier Na fixe la ligne rouge, c'est Ã  dire la coupure de l'allumage.
//Le nombre de points est libre.L'avance est imposÃ©e Ã  0Â° entre 0 et Nplancher t/mn
//Par exemple, courbe Savoy avec un " V " au dÃ©but pour favoriser le ralenti
int Na[] =      {0 ,500   ,1200     ,2000   ,5400  ,10000  ,0};//t/*mn vilo
int Anga[] =    {0 ,10    ,17      ,18     ,24    ,26     ,0};
int Ncyl = 1;           //Nombre de cylindres, moteur 4 temps.Multiplier par 2 pour moteur 2 temps

//__4__Position en degrÃ¨s vilo du capteur( Hall ou autre ) AVANT le PMH Ã©tincelle du cylindre NÂ°1
const int AngleCapteur = 108;

//__5__CapteurOn = 1 dÃ©clenchement sur front montant (par ex. capteur Hall "saturÃ©")
//CapteurOn = 0 dÃ©clenchement sur front descendant (par ex. capteur Hall "non saturÃ©").Voir fin du listing
const int CaptOn = 0;

//__6__Dwell = 1 pour alimenter la bobine en permanence sauf 1ms/cycle.Elle doit pouvoir chauffer sans crainte
//Dwell = 2 pour alimentation de la bobine seulement trech ms par cycle, 3ms par exemple
//Obligatoire pour bobine 'electronique'   de faible resistance: entre 2 et 0.5ohm.Ajuster  trech
//Dwell = 3 pour simuler un allumage Ã  vis platinÃ©es: bobine alimentÃ©e 2/3 (66%) du cycle
//Dwell = 4 pour optimiser l'Ã©tincelle Ã  haut rÃ©gime.La bobine chauffe un peu plus.
const int Dwell = 3;


//************************************************************************************
//**********************GENERALITES*********************************

//Valable pour tout type de capteur soit sur vilo soit dans l'allumeur (ou sur l'arbre Ã  came)
//La Led(D13) existant sur tout Arduino suit le courant dans la bobine:ON bobine alimentÃ©e
//En option, multi-Ã©tincelles Ã  bas rÃ©gime pour denoyer les bougies
//En option, compte-tours prÃ©cis
//En option, multi courbes , 2 courbes supplementaires, b et c, selectionables par D8 ou D9, ou smartphone
//En option, decalage de courbe 'au vol'
//Pour N cylindres,2,4,6,8,12,16, 4 temps, on a N cames d'allumeur ou  N/2 cibles sur le vilo
//Pour les moteurs Ã  1, 3 ou 5 cylindres, 4 temps, il FAUT un capteur dans l'allumeur (ou sur
//l'arbre Ã  cames, c'est la mÃªme chose)
//Exception possible pour un monocylindre 4 temps, avec  capteur sur vilo et une cible:on peut gÃ©nÃ¨rer
//une Ã©tincelle perdue au Point Mort Bas en utilisant la valeur Ncyl =2.
//Avance 0Â°jusqu'a Nplancher t/mn, anti retour de kick.

//**********************LES OPTIONS**********************

//*************Variantes de Dwell et Multi-Ã©ticelles**************
//Si Dwell=2, temps de recharge bobine, 3ms= 3000Âµs typique, 7ms certaines motos
const int trech  = 3000;
//Si Dwell=4, durÃ©e de l'Ã©tincelle tetin au delÃ  de Ntrans
const int Ntrans = 3000; //Regime de transition et maxi affichage RPM Sphone
const int tetin = 800; //Typique 500 Ã  1000Âµs, durÃ©e etincelle regimes >Ntrans
//Si multi-Ã©tincelle dÃ©sirÃ© jusqu'Ã  N_multi, modifier ces deux lignes
const int Multi = 1;//1 pour multi-Ã©tincelles
const int N_multi = 2300; //t/mn pour 4 cylindres par exemple
//Surtout utile pour denoyer les bougies Ã  bas rÃ©gime

//*********************Compte-tours prÃ©cis************************
//Connecter un HC05/06 Au +5V, masse, patte RX Ã  la patte 11 de l'Arduino, TX Ã  patte 10
//IMPORTANT: mettre le HC05/06 en mode 115200 bps via une commande AT
//http://www.loutrel.org/BlueToothArduino.html pour effectuer ce passage
//Sur le smartphone installer une appli telle que "Bluetooth Terminal HC-05"
//ou encore "BlueTerm+" ou equivallent.La premiÃ¨re fois seulement:
//inscrire le module sur le smartphone avec le code pin 1234..
//Ceci affiche sur un smartphone(ou tablette)Android  le rÃ©gime (t/mn /10) et l'avance en degrÃ¨s
//Surtout utile comme compte-tours precis pour regler la carburation au ralenti
#define Naff    20 //Pour affichage sur smartphone nb de tours Ã  ignorer entre deux affichages
//Maxi 3000t/mn
//*******************MULTICOURBES*******
// Mettre D8 ou D9 Ã  la masse pour selectionner la courbe b ou c
//ou utiliser un smartphone et un module Bluetooth

//*******//*********Courbe   b mettre D8 Ã  la masse ou entrer 8 avec Smartphone
//Courbe HervÃ©
int Nb[] = {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4200, 4600, 5100, 7000, 0};//t/*mn vilo
int Angb[] = {0, 8 , 10 , 22  , 24,    26,  28,    30,  30,   30,   32,    32,   32,  0};
//*******//*********Courbe   c mettre D9 Ã  la masse ou entrer 9 avec Smartphone
// C'est la courbe Savoy originale
int Nc[] = {0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4200, 4600, 5100, 7000, 0};//t/*mn vilo
int Angc[] = {0, 6 , 10 , 16  , 18,    22,  26,    28,  28,   28,   29,    30,   32,  0};

//***********************Decalage de courbe 'au vol'************************
//Pour jouer Ã  translater les courbes d'avance moteur en fonctionnement
//Avec un module Bluetooth HC05 ou 06
//Entrer 1 Ã  6 pour decaler la courbe en cours de 1 Ã  +6Â°
//Entrer 11 Ã  16 pour decaler la courbe en cours de -1 Ã  -6Â°
//Pour changer de courbe, entrer 8 pour basculer sur la courbe b, 9 pour  la c, et 7 pour la a
//Attention.....Pas de pleine charge avec trop d'avance, danger pour les pistons!!!

//************Ces 3 valeurs sont eventuellement modifiables*****************
//Ce sont: Nplancher, trech , Dsecu et delAv
const int Nplancher = 500; // vitesse en t/mn jusqu'a laquelle l'avance  = 0Â°
const int unsigned long Dsecu  = 1000000;//Securite: bobine coupee Ã  l'arret apres Dsecu Âµs

//*****************************************************************************
#include "TimerOne.h"
#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11);  //Creer une entrÃ©e/sortie sÃ©rie par logiciel sur D10 et D11
//***********************Variables du sketch************************************
const int Bob = A4;    //Sortie D4 vers bobine.En option, on peut connecter une Led avec R=330ohms vers la masse
const int Cible = A2;  //EntrÃ©e sur D2 du capteur, avec R PullUp
const int Pot = A0;   //EntrÃ©e analogique sur A0 pour potard de changement de courbes, avec R PullUp
const int Led13 = 13; //Temoin sur tout Arduino, suit le courant de bobine
const int Courbe_b = 8;  //EntrÃ© D8  R PullUp.Connecter Ã  la masse pour courbe b
const int Courbe_c = 9;  //EntrÃ© D9  R PullUp. Connecter Ã  la masse pour courbe c
//PotentiomÃ¨tre de decalage Ã  3 positions:Decalage 0Â° si <1V, delAVÂ° si 1 Ã  2 V, 2*delAvÂ° pour > 2V
int valPot = 0;       //0 Ã  1023 selon la position du potentiomÃ¨tre en entree
int delPot = 2;      // le pas de decalage en degrÃ¨s
float modC1 = 0;      //Correctif pour C1[], deplace la courbe si potard connectÃ©
int unsigned long D = 0;  //Delai en Âµs Ã  attendre aprÃ¨s passage de la cible pour l'Ã©tincelle
int milli_delay = 0;
int micro_delay = 0;
float RDzero = 0; //pour calcul delai avance 0Â° < Nplancher t/mn
float  Tplancher = 0; //idem
int tcor  = 140; //correction en Âµs  du temps de calcul pour D
int unsigned long Davant_rech = 0;  //Delai en Âµs avant la recharge de la  bobine.
int unsigned long prec_H  = 0;  //Heure du front precedent en Âµs
int unsigned long T  = 0;  //Periode en cours
int unsigned long Tprec  = 0;//Periode precedant la T en cours, pour calcul de Drech
int N1  = 0;  //Couple N,Ang de debut d'un segment
int Ang1  = 0; //Angle d'avance vilo en degrÃ¨s
int N2  = 0; //Couple N,Ang de fin de segment
int Ang2  = 0;
int*  pN = &Na[0];//pointeur au tableau des rÃ©gimes. Na sera la courbe par defaut
int*  pA = &Anga[0];//pointeur au tableau des avances. Anga sera la  courbe par defaut
float k = 0;//Constante pour calcul de C1 et C2
float C1[30]; //Tableaux des constantes de calcul de l'avance courante
float C2[30]; //Tableaux des constantes de calcul de l'avance courante
float Tc[30]; //Tableau des Ti correspondants au Ni
//Si necessaire, augmenter ces 3 valeurs:Ex C1[40],C2[40],Tc[40]
int Tlim  = 0;  //PÃ©riode minimale, limite, pour la ligne rouge
int j_lim = 0;  //index maxi des N , donc aussi  Ang
int unsigned long NT  = 0;//Facteur de conversion entre N et T Ã  Ncyl donnÃ©
int unsigned long NTa  = 0;//Facteur de conversion entre N et T pour affichage sur smartphone
int ctNaff = 0; // Compteur de Naff
int AngleCibles = 0;//Angle entre 2 cibles, 180Â° pour 4 cyl, 120Â° pour 6 cyl, par exemple
int UneEtin = 1; //=1 pour chaque Ã©tincelle, testÃ© et remis Ã  zero par isr_GestionIbob()
int Ndem = 200;//Vitesse estimÃ©e du vilo entrainÃ© par le demarreur en t/mn
int unsigned long Tdem  = 0;  //Periode correspondante Ã  Ndem,forcÃ©e pour le premier tour
int Mot_OFF = 0;//Sera 1 si moteur detectÃ© arrÃ©tÃ© par l'isr_GestionIbob()
int unsigned long Ttrans; //T transition de Dwell 4
int unsigned long T_multi  = 0;  //Periode minimale pour multi-Ã©tincelle
//Permet d'identifier le premier front et forcer T=Tdem, ainsi que Ibob=1, pour demarrer au premier front
String  Ligne; //Stock une ligne du smartphone,ex "5" pour augmenter l'avance de 5Â°
char  carLu;//Sera cumulÃ© dans Ligne
int delAv = 7;  //Donne la courbe en cours 7 ou 8 ou 9, Code de decalage 1 Ã  6 Â°,ou alors
//choix de courbe demandÃ© au Sphone: 7 courbe a, 8 courbe b, 9 courbe c
int Ncourbe = 7;  //Numero de la courbe en cours Ã  afficher, au demarrage c'est 7 donc courbe a
//********************LES FONCTIONS*************************

void  CalcD ()//////////////////
// Noter que T1>T2>T3...
{ for (int j = 1; j <= j_lim; j++)//On commence par T la plus longue et on remonte
  {
    if  (T >=  Tc[j]) {     //on a trouvÃ© le bon segment de la courbe d'avance
      D =  float(T * ( C1[j] - modC1 )  + C2[j]) ;//D en Âµs, C2 incorpore le temps de calcul tcor
      if ( T > Tplancher)D = T * RDzero;//Imposer 0Â° d'avance de 0 Ã  500t/mn
      break;  //Sortir, on a D
    }
  }
}
void  Etincelle ()//////////
{ if (D < 14000) {         // 16383 Âµs semble Ãªtre le maxi pour la fonction  delayMicroseconds(D)
    delayMicroseconds(D); //Attendre D }
  }
  else {
    milli_delay = ((D / 1000) - 2);//Pour ces D longs, delayMicroseconds(D)ne va plus.
    micro_delay = (D - (milli_delay * 1000));
    delay(milli_delay); //
    delayMicroseconds(micro_delay);
  }
  digitalWrite(Bob, 0);//Couper le courant, donc Ã©tincelle
  digitalWrite(Led13, 0); //Temoin
  //Maintenant que l'Ã©tincelle est Ã©mise, il faut rÃ©tablir Ibob au bon moment

  if (Multi && (T >= T_multi))Genere_multi();  //Voir si multi Ã©tincelle demandÃ©e
  else {
    switch (Dwell)  //Attente courant coupÃ© selon le type de Dwell

    { case 1:       //Ibob coupe 1ms par cycle seulement, la bobine doit supporter de chauffer
        Davant_rech = 1000; //1ms off par cycle
        break;

      case  2:      //Type bobine faible resistance, dite "electronique"
        Davant_rech = 2 * T - Tprec - trech;//On doit tenir compte des variations de rÃ©gime moteur
        Tprec = T;    //Maj de la future periode precedente
        break;

      case  3:      //Type "vis platinÃ©es", Off 1/3, On 2/3
        Davant_rech = T / 3;
        break;


      case  4:     //Type optimisÃ© haut rÃ©gime
        if ( T > Ttrans )Davant_rech = T / 3; // En dessous de N trans, typique 3000t/mn
        else Davant_rech = tetin; // Au delÃ  de Ntrans, on limite la durÃ©e de l'Ã©tincelle, typique 0.5ms
        break;
    }
    Timer1.initialize(Davant_rech);//Attendre Drech Âµs avant de retablire le courant dans la bobine
  }
  UneEtin = 1; //Pour signaler que le moteur tourne Ã  l'isr_GestionIbob().
  if (T > Ttrans)Smartphone();  //Si pas trop vite gÃ©rer le sphone
}
void  Genere_multi()//////////
{ //L'etincelle principale a juste Ã©tÃ© gÃ©nÃ©rÃ©e
  delay(1); //Attendre fin d'etincelle 1ms
  digitalWrite(Bob, 1);//Retablir  le courant
  delay(3); //Recharger 3ms
  digitalWrite(Bob, 0);//PremiÃ¨re etincelle secondaire
  delay(1); //Attendre fin d'etincelle 1ms
  digitalWrite(Bob, 1);//Retablir  le courant
  delay(2); //Recharger 2 ms
  digitalWrite(Bob, 0);//DeuxiÃ¨me etincelle secondaire
  delay(1); //Attendre fin d'etincelle 1ms
  digitalWrite(Bob, 1);//Retablir  le courant pour Ã©tincelle principale
}
void  Init ()/////////////
//Calcul de 3 tableaux,C1,C2 et Tc qui serviront Ã  calculer D, temps d'attente
//entre la detection d'une cible par le capteur  et la generation de l'etincelle.
//Le couple C1,C2 est determinÃ© par la periode T entre deux cibles, correspondant au
//bon segment de la courbe d'avance entrÃ©e par l'utilisateur: T est comparÃ©e Ã  Tc
{ AngleCibles = 720 / Ncyl; //Cibles sur vilo.Ex pour 4 cylindres 180Â°,  120Â° pour 6 cylindres
  NT  = 120000000 / Ncyl; //Facteur de conversion Nt/mn moteur, TÂµs entre deux PMH Ã©tincelle
  //c'est Ã  dire deux cibles sur vilo ou deux cames d'allumeur: Nt/mn = NT/TÂµs
  NTa = NT / 10; ///Facteur de conversion Nt/mn moteur pour afficher N/10 au smartphone
  Ttrans = NT / Ntrans; //Calcul de la periode de transition pour Dwell 4
  T_multi = NT / N_multi; //Periode minimale pour generer un train d'Ã©tincelle
  modC1 = 0;
  Tdem  = NT / Ndem; //Periode imposÃ©e Ã   la premiÃ¨re Ã©tincelle qui n'a pas de valeur prec_H
  Tplancher = 120000000 / Nplancher / Ncyl; //T Ã   vitesse plancher en t/mn: en dessous, avance centrifuge = 0
  RDzero = float(AngleCapteur) / float(AngleCibles);
  Prep_Courbe();  //Calcul les segments de la courbe d'avance specifiÃ©e par delAv
  //  Serial.print("Ligne_"); Serial.println(__LINE__);
  //  Serial.print("Tc = "); for (i = 1 ; i < 15; i++)Serial.println(Tc[i]);
  //  Serial.print("Tlim = "); Serial.println(Tlim);
  //  Serial.print("C1 = "); for (i = 1 ; i < 15; i++)Serial.println(C1[i]);
  //  Serial.print("C2 = "); for (i = 1 ; i < 15; i++)Serial.println(C2[i]);
  //Timer1 a deux roles:
  //1)couper le courant dans la bobine en l'absence d'etincelle pendant plus de Dsecu Âµs
  //2)aprÃ¨s une Ã©tincelle, attendre le delais Drech avant de retablir le courant dans la bobine
  //Ce courant n'est retabli que trech ms avant la prochaine Ã©tincelle, condition indispensable
  //pour une bobine Ã  faible resistance, disons infÃ©rieure Ã  3 ohms.Typiquement trech = 3ms Ã  7ms
  Timer1.attachInterrupt(isr_GestionIbob);//IT d'overflow de Timer1 (16 bits)
  Timer1.initialize(Dsecu);//Le courant dans la bobine sera coupÃ© si aucune etincelle durant Dsecu Âµs
  Mot_OFF = 1;// Signalera Ã  loop() le premier front
  digitalWrite(Bob, 0); //par principe, couper la bobine
  digitalWrite(Led13, 0); //Temoin
}
void  isr_GestionIbob()//////////
{ Timer1.stop();    //Arreter le decompte du timer
  if (UneEtin == 1) {
    digitalWrite(Bob, 1);    //Le moteur tourne,retablire le courant dans bobine
    digitalWrite(Led13, 1);//Temoin
  }
  else
  { digitalWrite(Bob, 0);  digitalWrite(Led13, 0); //Temoin//Moteur arrete, preserver la bobine, couper le courant
    Mot_OFF = 1;//Permettra Ã  loop() de detecter le premier front de capteur
  }
  UneEtin = 0;  //Remet  le detecteur d'Ã©tincelle Ã  0
  Timer1.initialize(Dsecu);//Au cas oÃ¹ le moteur s'arrete, couper la bobine apres Dsecu Âµs
}
void Lect_delAv()///////////////////Uniquement si T> Ttrans, typique N <3000t/mn
//Lecture du smartphone
{ // Avec 7,8 ou 9 on change de courbe, sinon delta de +1 deg Ã  +6 deg  ou negatif 11 Ã  16
  // Serial.println( "enter Lect delAv");
  Ligne = ""; //On y accumule les car reÃ§us du sphone
  while (BT.available() > 0) //Nb de car dans le buffer venant du sphone
  { carLu = BT.read(); //oui il y a un car au moins, mais on filtre 1 Ã  9 seulement, on se mÃ©fie
    if ((carLu >= 49 ) && (carLu <= 57) )Ligne = Ligne + carLu; //si entre 1 et 9
  }
  Serial.println( Ligne);
  if (Ligne != "") //Une valeur a Ã©tÃ© entrÃ©e au sphone
  { delAv = Ligne.toInt(); // convertir en entier
    Serial.println( delAv);
    if ((delAv >= 1) && (delAv <= 16))
      Serial.println( delAv);
    {
      if ((delAv == 8) || (delAv == 9) || (delAv == 7)) {
        {
          Prep_Courbe(); //On change de courbe, appel Select_Courbe()
          Ncourbe = delAv; //Pour affichage
        }
      }
      else          //Delta sur courbe en cours de +1 a +6 deg ou negatif
      {
        if (delAv >= 11)delAv = 10.0 - delAv; // par ex  on entre 12, pour -2Â°
        modC1 = float(delAv) / float(AngleCibles); // pour le calcul du delai avant Ã©tincelle
        BT.println(delAv); // Afficher sur sphone
      }
    }
  }
}
void  Prep_Courbe()/////////Calcul les segments de la courbe d'avance
{ //delAv =  7,8,ou 9 pour la courbe Ã  activer, a ou b ou c
  Select_Courbe();  //Ajuster  les pointeurs pN et pA pour la courbe designÃ©e par delAv
  N1  = 0; Ang1 = 0; //Toute courbe part de  0
  int i = 0;    //locale mais valable hors du FOR
  pN++; pA++; //sauter le premier element de tableau, toujours =0
  for (i  = 1; *pN != 0; i++)//i pour les C1,C2 et Tc.Arret quand regime=0.
    //pN est une adresse (pointeur) qui pointe au tableau N.Le contenu pointÃ© est *pN
  { N2 = *pN; Ang2 = *pA;//recopier les valeurs pointÃ©es dans N2 et Ang2
    k = float(Ang2 - Ang1) / float(N2  - N1);//pente du segment (1,2)
    C1[i] = float(AngleCapteur - Ang1 + k * N1) / float(AngleCibles);
    C2[i] = -  float(NT * k) / float(AngleCibles) - tcor; //Compense la durÃ©e de calcul de D
    Tc[i] = float(NT / N2);  //
    N1 = N2; Ang1 = Ang2; //fin de ce segment, dÃ©but du suivant
    pN++; pA++;   //Pointer Ã  l'element suivant de chaque tableau
  }
  j_lim = i - 1; //Revenir au dernier couple entrÃ©
  Tlim  = Tc[j_lim]; //Ligne rouge
}
void  Select_Courbe()///////////
//Initialise pN et pA vers  une des 3 courbes a , b , c selon delAv
{ //Serial.println("********************************delAv**********");
  //Serial.println( delAv);
  if (delAv == 7) //C'est la  courbe d'origine par dÃ©faut, a
  { pN = &Na[0];
    pA = &Anga[0];
  }
  if ((digitalRead(Courbe_b) == 0) || (delAv == 8))  //D8 Ã  la masse ou 8 au Sphone
  { pN = &Nb[0];  // pointer Ã  la courbe b
    pA = &Angb[0];
  }
  if ((digitalRead(Courbe_c) == 0) || (delAv == 9))  //D9 Ã  la masse ou 9 au Sphone
  { pN = &Nc[0];  // pointer Ã  la courbe c
    pA = &Angc[0];
  }
}
void  Smartphone()///////////////////////////////////////////
{ //Si  N < Ntrans on affiche en Bluetooth le regime, l'avance et possibilitÃ© de decaler ou changer de courbe

  if (ctNaff < Naff)ctNaff++; // typiquement on saute 5 Ã  10  tours entre 2 affichages de N
  else
  { //Afficher vitesse, avance et NÂ° de courbe
    BT.print(NT / T);  //Afficher N et avance sur smartphone
    BT.print("................deg  ");
    BT.println(AngleCapteur - (D + tcor)*AngleCibles / T);//Afficher avance
    BT.print("Courbe ");
    BT.println(Ncourbe);  //Afficher 7 ou 8 ou 9  pour la courbe en cours
    BT.println();
    ctNaff = 0; //RAZ du compteur d'affichages
    //Deux options
    Tst_Pot();//Voir si un potard connectÃ© pour decaler la courbe
    Lect_delAv();//Voir si decalage ou basculement de courbe demandÃ©e au sphone
  }
}
void  Tst_BT()
{ int BTdata = 0;
  //**************Vers PC
  Serial.println("Bonjour ");
  Serial.println("Le caractere entre sur l'Android doit se retrouver ci dessous");
  //***************Vers module
  BT.println(" ");
  BT.println(" ");
  BT.println("Bonjour sur Android");
  BT.println(" ");
  BT.println("HC05, la led doit clignoter 2 flash, 1s Off, 2flash...");
  BT.println(" ");
  BT.println("HC06, led On fixe");
  BT.println("");
  BT.println("Entrer un caractere ");
  BT.println("S'il se retouve sur le PC, tout va bien...");
  while (1)
  { if (BT.available()) //Car pret en entrÃ©e sur Serie soft?
    { BTdata = BT.read(); //oui, le clavier Android BT a Ã©mis un car
      BT.println(" ");//Aller Ã  la ligne sur sphone

      // **************Vers PC
      Serial.write(BTdata);//ecrire le car sur le PC
      Serial.println();
    }
  }
}
void Tst_Pot()///////////
{ valPot = analogRead(Pot);
  if (valPot < 240 || valPot > 900);//0Â° ou pas de potard connectÃ© (valpot =1023 en thÃ©orie)
  else {
    if (valPot < 500)modC1 = float (delPot) / float(AngleCibles);//Position 1
    else modC1 = 2 * float (delPot) / float(AngleCibles);//Position 2
  }
}
////////////////////////////////////////////////////////////////////////
void setup()///////////////
/////////////////////////////////////////////////////////////////////////
{
  unsigned long t = millis();
  Serial.begin(115200);//Ligne suivante, 3 Macros du langage C
  Serial.println(__FILE__); Serial.println(__DATE__); Serial.println(__TIME__);
  BT.begin(115200);//Vers module BlueTooth HC05/06
  BT.flush();//A tout hasard
  BT.println(__FILE__); BT.println(__DATE__); BT.println(__TIME__);
  BT.println("***Bonjour*****************************************");
  BT.println("******************************************************");
  pinMode(Cible, INPUT_PULLUP); //EntrÃ©e front du capteur sur D2
  pinMode(Bob, OUTPUT); //Sortie sur D4 controle du courant dans la bobine
  pinMode(Pot, INPUT_PULLUP); //EntrÃ©e pour potard 100kohms, optionnel
  pinMode(Courbe_b, INPUT_PULLUP); //EntrÃ©e Ã  la masse pour selectionner la courbe b
  pinMode(Courbe_c, INPUT_PULLUP); //EntrÃ©e Ã  la masse pour selectionner la courbe c
  pinMode(Led13, OUTPUT);//Led d'origine sur tout Arduino, temoin du courant dans la bobine
  // Tst_BT();  //Option pour tester le Bluetooth, boucle ici,  sinon Ã  commenter.
  Init();// ExecutÃ©e une fois au demarrage
}
///////////////////////////////////////////////////////////////////////////
void loop()   ////////////////
////////////////////////////////////////////////////////////////////////////
{ while (digitalRead(Cible) == !CaptOn); //Attendre front actif de la cible
  T = micros() - prec_H;    //front actif, arrivÃ© calculer T
  prec_H = micros(); //heure du front actuel qui deviendra le front precedent
  if ( Mot_OFF == 1 ) { //Demarrage:premier front de capteur
    T = Tdem;//Fournir  T = Tdem car prec_H n'existe par pour la premiÃ¨re Ã©tincelle
    digitalWrite(Bob, 1);//Alimenter la bobine
    digitalWrite(Led13, 1); //Temoin
    Mot_OFF = 0; //Le moteur tourne
  }
  if (T > Tlim)     //Sous la ligne rouge?
  { CalcD(); //Oui, generer une etincelle
    Etincelle();
  }
  while (digitalRead(Cible) == CaptOn); //Attendre si la cible encore active
}
