
# S4H2022-RufUS/BASE
## [Fichers CAD](https://github.com/Beam-create/S4H2022-RufUS/tree/main/CAD)
Cette section contient tous les fichiers CAD de la base et se divise selon les trois sections suivantes:
### CAD_SIM
Cette section contient les pièces et les assemblages SolidWorks des composants primaires de la base et de ses roues (fichiers SLDPRT et SLDASM).
### DXF
Cette section contient les fichiers DXF de la base à découper au laser
### STL
Cette section contient les fichiers STL de composants secondaires de la base, tel que des supports, monture, etc.

## Comment fabriquer la base mobile
![Base_mec_cool](https://user-images.githubusercontent.com/72213923/155202295-f0e4fc5b-6d43-4b49-9942-6be311430c08.jpg)

La base mobile est faite de matériaux et de composants faciles à trouver. Sa réalisation nécessite l'accès à une **découpeuse laser à bois**, une **imprimante 3d** et des outils électroniques tels qu'un **fer à souder**, un **multimètre** et une **source d'alimentation 12V** (optionnel).

### Liste des matériaux et composantes
***
#### Mécanique
- [ ]  Vis M3 X 0.5 
- [ ] Écrous M3 X 0.5 
- [ ] 2' X 3' X 1/4" contre-plaqué
- [ ] Roue Mecanum 80mm ([robotshop](https://www.robotshop.com/ca/en/80mm-mecanum-wheel-kit-4x.html))
- [ ] Moteur Pololu 50:1 12VDC  ([pololu](https://www.pololu.com/product/4753))
- [ ] Moyeu Servocity 1/4"  ([servocity](https://www.servocity.com/0-250-0-770-set-screw-hub/))
- [ ] Support moteur Pololu ([pololu](https://www.pololu.com/product/1084))

### Électrique
- [ ] Arduino Mega 2560 ([arduino](https://store-usa.arduino.cc/products/arduino-mega-2560-rev3))
- [ ] Contrôleur Cytron MD13s ([cytron](https://www.cytron.io/p-13amp-6v-30v-dc-motor-driver))
- [ ] Convertisseur DC-DC ([amazon](https://www.amazon.ca/Yizhet-Module-dalimentation-r%C3%A9glable-LM2596/dp/B08Q2YKJ6Q/ref=sr_1_4_sspa?keywords=variable+buck+converter&qid=1645588038&sprefix=variable+buck%2Caps%2C87&sr=8-4-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyTzZQOVVRQUg1VTJYJmVuY3J5cHRlZElkPUEwMTczNjA2SlRaNlpGSUtMRkxFJmVuY3J5cHRlZEFkSWQ9QTA4NTUxOTYyQVM3Vjk5TzNHM0gyJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==))
- [ ] Ensemble connecteurs Dupont ([amazon](https://www.amazon.ca/Yangoutool-Dupont-connecteurs-cliquet-JST-XH/dp/B07X9MJ8G2/ref=sr_1_3_sspa?__mk_fr_CA=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=22K7F4Z52OQK1&keywords=jst+connector+kit&qid=1645481928&sprefix=jst+connector+kit%2Caps%2C71&sr=8-3-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFFVjVON0VMUDJCRUImZW5jcnlwdGVkSWQ9QTA0OTM2NDYzUFdHRDM1M1NMMUROJmVuY3J5cHRlZEFkSWQ9QTAwMjk1ODM5VlFUSjM0OTNUS0Emd2lkZ2V0TmFtZT1zcF9hdGYmYWN0aW9uPWNsaWNrUmVkaXJlY3QmZG9Ob3RMb2dDbGljaz10cnVl))
- [ ] Ensemble espaceur PCB ([amazon](https://www.amazon.ca/Litorange-entretoises-hexagonales-femelle-circuit/dp/B07TP2YYQB/ref=sr_1_1_sspa?__mk_fr_CA=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3PX4RNR4ABLM5&keywords=pcb+spacer&qid=1645482172&sprefix=pcb+spacer%2Caps%2C74&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyMzhTRTFZSEFKTVYzJmVuY3J5cHRlZElkPUExMDMyMzczUlg1QkE5RUk1STRVJmVuY3J5cHRlZEFkSWQ9QTA3NTI5NDUyWEg1OUNPOEtOWDhXJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==))
- [ ] Fil 22 awg ([robotshop](https://www.robotshop.com/ca/fr/kit-fils-pvc-noyau-solide-plusivo-22awg-6-couleurs-10-m-chacun.html))
- [ ] Pin header([amazon](https://www.amazon.ca/-/fr/10-Paire-femelle-broches-Connecteur-Arduino/dp/B019WOPOHI/ref=sr_1_10?__mk_fr_CA=ÅMÅŽÕÑ&crid=2QZMUE8YLUWN3&keywords=pin+header&qid=1649268265&sprefix=pin+header%2Caps%2C54&sr=8-10))
- [ ] Connecteur bornier([amazon](https://www.amazon.ca/-/fr/gp/product/B07H5G7GC6/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&th=1))
- [ ] Connecteur grove([amazon](https://www.amazon.ca/-/fr/WOWOONE-connecteurs-femelles-Micro-broches/dp/B08RHGT3W3/ref=sr_1_38?__mk_fr_CA=ÅMÅŽÕÑ&crid=2IVHVETWWB96F&keywords=grove+connectors+4&qid=1649268329&sprefix=grove+connectors+4%2Caps%2C57&sr=8-38))

### Assemblage mécanique
***
Ce [dossier](https://github.com/Beam-create/S4H2022-RufUS/tree/main/CAD) contient plus de détails sur l'assemblage, ainsi que les STL et DXF pour fabriquer les pièces.
#### Assemblage châssis
![S4H2022_Base_000](https://user-images.githubusercontent.com/72213923/155043265-a48b9cb9-ba62-4f9d-9cca-9c9b88f82a11.JPG)

#### Assemblage roues
![S4H2022_Base_wheel_assy](https://user-images.githubusercontent.com/72213923/155043038-8b1f1ff6-de8e-483d-bc60-c8016de359c1.JPG)

### Assemblage électrique
***
![Base_elec](https://user-images.githubusercontent.com/72213923/155202543-286f9da8-20ac-4cc9-b509-e796d946be58.jpg)

#### Circuit imprimé
![image](https://user-images.githubusercontent.com/54538310/162044831-c107e6bc-2753-42b0-91d1-e67a90bdf6a5.png)


Un PCB a été concu afin de facilliter les branchements électriques. Toute l'information reliée au sens des branchements est indiqué directement sur le circuit. Pour se procurer le PCB, il suffit d'aller sur un site de commande de pcb en ligne (ex: [jlcpcb.com](https://jlcpcb.com/VGB?gclid=CjwKCAjw9LSSBhBsEiwAKtf0n9PKT3Qgz4o0cPbz0CCLisM4oUogkxZFzJCL4dg2W4-eoO5GJ_axChoCpaEQAvD_BwE)) et d'y téléverser le dossier compressé  [*PCB_Production.zip*](https://github.com/Beam-create/S4H2022-RufUS/files/8502373/PCB_Production.zip). Aucun paramètre par défaut n'est à modifier.



> **IMPORTANT**: Pour la section de pin à souder située à droite du Arduino Mega, il faut SEULEMENT souder les pins entourées en rouge.

##### Configuration des convertisseurs DC-DC
1. Brancher une source de tension (12V) à l'entré du convertisseur et brancher un multimetre à sa sortie
2. À l'aide d'un tournevis, vous devez changer la tension de sortie à 5V et à 6.8V pour deux convertisseurs que vous pourrez ensuite placer sur le pcb aux endroits indiqués.
