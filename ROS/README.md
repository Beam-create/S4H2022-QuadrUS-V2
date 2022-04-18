# S4H2022-RufUS/ROS

## Configuration du Jetson Nano
### Création d'un hotspot
La création d'un réseau wi-fi dit "hotspot" sur un appareil permet à un ordinateur tiers de se connecter à cet appareil et d'avoir accès à l'internet sans fil si l'appareil recoit une connexion internet filaire. Par extension, cela permet à l'ordinateur tiers d'accéder au terminal de l'appareil par "ssh" afin de lancer des commandes (voir la section correspondante). Le but ici est donc de lancer les commandes ROS sur le Jetson Nano à partir d'un autre ordinateur sans connexion filaire reliant les deux appareils.
##### Méthode 1 (simple)
1. Aller dans la section "Wi-Fi" des paramètres de Ubuntu du Jetson Nano
2. Appuyer sur les trois petits points en haut à droite de la fenêtre, puis sélectionner "Allumer le point d'accès Wi-Fi..."
![image](https://user-images.githubusercontent.com/72227100/158654582-d875c6a9-51ba-4a0b-b354-033e0b6df990.png)
3. Assigner un nom au réseau (au choix), définir un mot de passe (au choix) et appuyer sur "Allumer"
![image](https://user-images.githubusercontent.com/72227100/158654909-0f6306aa-b62a-47df-a4e2-0428e4f4a491.png)
4. Vérifier si le hotspot apparait parmi les réseaux disponible sur l'ordinateur, le cas échéant, s'y connecter

#### Méthode 2 (plus d'options)
1. Ouvrir un terminal de commande sur le Jetson Nano, puis taper la commande "nm-connection-editor": la fenêtre "Connexions réseau" devrait apparaître
![image](https://user-images.githubusercontent.com/72227100/158657653-87b1e54c-5243-4ddb-9c61-2f2e6a94f022.png)
2. Appuyer sur "+" en bas à gauche de la fenêtre pour ajouter un nouveau réseau
![image](https://user-images.githubusercontent.com/72227100/158657894-5df9a737-e0b3-4ca3-a02c-a2a28054d839.png)
3. Sélectionner "Wi-Fi" dans les options de la fenêtre "Sélectionner un type de connexion", puis appuyer sur "Créer"
![image](https://user-images.githubusercontent.com/72227100/158658068-a4d7c36f-70ac-4548-9407-dd8d8604e167.png)
4. Modifier au minimum ces paramètres dans l'onglet "Wi-Fi":
	*   Assigner un nom à la connexion (au choix)
	*  Assigner un SSID (au choix)à
	* Sélectionner le mode "Hotspot"
![image](https://user-images.githubusercontent.com/72227100/158658275-4d16baf1-97ac-4aad-9bb6-327275073e60.png)
5. Optionnel: Modifier les autres paramètres selon les besoins
6. Appuyer sur "Enregistrer"
7. Aller dans la section "Wi-Fi" des paramètres de Ubuntu du Jetson Nano
8. Appuyer sur les trois petits points en haut à droite de la fenêtre, puis sélectionner "Connexion à un réseau masqué..."
![image](https://user-images.githubusercontent.com/72227100/158658469-33a2f0fa-6f90-41c7-87d7-632286de9e85.png)
9. Sélectionner le hotspot créé en fonction du nom assigné à l'étape 4, puis appuyer sur "Se connecter"
![image](https://user-images.githubusercontent.com/72227100/158658654-0ac53ffa-0ce1-497c-b9ce-2fc50742b386.png)
10. Vérifier si le hotspot apparait parmi les réseaux disponible sur l'ordinateur, le cas échéant, s'y connecter

### Connexion au Jetson Nano par SSH
1. Ouvrir un terminal de commande sur le Jetson Nano et taper "ifconfig"
2. Noter l'adresse ip à côté de "inet" sous "wlan0" de la forme 10.x.x.x, par exemple 10.42.0.1
3. Ouvrir un terminal de commande sur l'ordinateur et taper "ssh <Nom du Jetson Nano>@<Adresse ip notée>", par exemple "ssh jetson@10.42.0.1"
4. Accepter les demandes d'accès et entrer le mot de passe du Jetson Nano le cas échéant
