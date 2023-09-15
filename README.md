# DMX_ESP_Stepper-controller

controle de moteur pas à pas selon commande DMX

Clément SAILLANT pour Hémisphère - 2023
c.saillant@gmail.com
0625334420

Un canal DMX est utilisé pour la commande de la position du moteur
Un autre canal DMX est utilisé pour la commande de la vitesse du moteur
Une fonction affine est utilisée pour convertir la valeur DMX en position ou vitesse
La position et la vitesse sont stockées dans des variables globales
Une fonction est utilisée pour la sécurité si un obstacle est détecté par la perte de pas du moteur

moteur : 34HS59-6004D-E1000
Angle de pas: 1.8 deg
Résolution de l'encodeur: 1000PPR(4000CPR)
https://www.omc-stepperonline.com/download/34HS59-6004D-E1000_Torque_Curve.pdf
https://www.omc-stepperonline.com/download/34HS59-6004D-E1000.pdf

Controleur CL86T V4.0
https://www.omc-stepperonline.com/index.php?route=product/product/get_file&file=389/CL86T%20(V4.0).pdf

<img width="545" alt="3D view" src="https://github.com/electron-rare/DMX_ESP_Stepper-controller/assets/108685187/e1c40d03-097e-4f09-b654-c9919626e945">
