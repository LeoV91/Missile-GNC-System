# Segway Control System — LQI Control Project

## Overview

Ce projet présente la modélisation et la synthèse d'une loi de commande classique pour un système de type **Segway** (pendule inversé sur roue motorisée), implémenté en **MATLAB R2025a** et **Simulink**.

Ce projet combine :
- La modélisation du Segway (réalisée dans [`modelisation.md`](modelisation1.md))
- La synthèse d'une loi de commande par retour d'état optimal **LQI** (LQR + action intégrale) assurant le suivi d'une consigne de position
- L'analyse de stabilité, de performance et des limites physiques du système en boucle fermée
- *(Piste future)* l'extension vers une architecture **LQG** avec observateur, et l'intégration dans un scénario de visualisation 3D

L'objectif est de stabiliser un système intrinsèquement instable en boucle ouverte (pendule inversé) et de le faire suivre une consigne de position `x(t)`, tout en respectant les limites physiques de l'actionneur (couple moteur) et les contraintes structurelles du système.

> **Statut du document** : les sections 1 à 2 et la synthèse LQI (section 3) reflètent le travail effectivement réalisé et validé (script MATLAB + Simulink). Les sections concernant l'extension LQG (observateur, bruit) et l'intégration 3D sont pour l'instant des pistes de travail futures, marquées comme telles ci-dessous — elles n'ont pas encore été implémentées.

## Documentation
- [Segway_Model](Segway-Modeling-1.md) — dérivation complète des équations du mouvement et linéarisation

## 1. System Modeling

La modélisation du système jusqu'à l'obtention d'une représentation d'état a été réalisée dans la partie *Segway_Model*. Se référer à ce document pour le détail du cheminement (formalisme de Newton-Euler / Lagrange) et la détermination des équations. Cette section se contente d'un résumé, en vue de l'implémentation de la loi de commande LQ.

### 1.1 State-Space Representation

**États** : `X = [x, x_dot, theta, theta_dot]'`
- `x` : position de la roue [m]
- `theta` : angle d'inclinaison du châssis [rad]

**Entrée** : `u = T`, le couple moteur appliqué à la roue [N.m]

**Limitation de l'entrée** : en l'absence de fiche technique moteur définitive, une limite de couple de l'ordre de **3 N.m maximum** est retenue comme hypothèse de dimensionnement cohérente avec un système de cette taille (M = m = 0.3 kg) — typique d'un motoréducteur DC pour robot compact. *Cette valeur doit être confirmée/ajustée selon le moteur réellement sélectionné.*

**Sortie régulée** : `y = x` (position de la roue)

**Modèle linéarisé** (autour du point d'équilibre instable `theta = 0`) :

```
A = [0        1        0                          0;
     0        0        3*m*g/(4*M+m)               0;
     0        0        0                          1;
     0        0        6*g*(M+m)/(L*(4*M+m))       0]

B = [0; 4/(4*M+m); 0; 6/(L*(4*M+m))]

C = [1 0 0 0]
D = [0]
```

avec `M = m = 0.3 kg`, `L = 0.67 m`, `g = 9.81 m/s²`.

### 1.2 Open-Loop Analysis

Les pôles en boucle ouverte (valeurs propres de `A`) sont :

| Pôle | Interprétation |
|---|---|
| `0` (double) | intégration pure (position ← vitesse) |
| `+5.928 rad/s` | **mode instable** — divergence exponentielle de l'inclinaison |
| `-5.928 rad/s` | mode stable symétrique |

La présence d'un pôle à partie réelle strictement positive confirme que **le système est instable en boucle ouverte** — attendu pour un pendule inversé. La fonction de transfert `u → x` présente en outre un **zéro de transmission à phase non minimale** en `s = +4.686 rad/s` (voir §3.4) : physiquement, accélérer la roue vers l'avant nécessite d'abord une inclinaison du châssis vers l'arrière. Cette propriété structurelle, invariante par retour d'état, borne les performances atteignables en boucle fermée quelle que soit l'agressivité des gains (cf. §3.4).

## 2. LQR Control Design

### 2.1 Control Objective

Stabiliser le système autour de `theta = 0` tout en asservissant la position `x` à une consigne `r(t)` variable dans le temps (échelon, puis retour à zéro, puis segment sinusoïdal — voir profil `Rcmd` généré dans le script). Un simple retour d'état LQR régule le système vers l'origine mais ne garantit pas d'erreur statique nulle pour une consigne non nulle ; une action intégrale est donc nécessaire (§3).

### 2.2 LQR Gain and Stability

Détermination du gain `K` par résolution de l'équation de Riccati (`lqr(A,B,Q,R)`), avec vérification préalable de la commandabilité et de l'observabilité du système (`rank(ctrb(A,B))`, `rank(obsv(A,C))`) — condition nécessaire à l'existence d'une solution stabilisante.

Une précompensation statique `N = -1/(C*((A-B*K)\B))` a été envisagée pour annuler l'erreur statique, mais s'avère **inadaptée** dès que la sortie régulée change (par ex. `y = x_dot`) ou en présence de perturbations — d'où le choix d'une architecture LQI (§3).

### 2.3 Closed-Loop Dynamics

Voir les résultats détaillés en §3 (le retour d'état LQR seul, sans action intégrale, n'a été utilisé que comme étape intermédiaire de vérification ; l'architecture retenue pour la commande finale est LQI).

## 3. LQI Control Architecture

Le suivi de consigne est assuré par une action intégrale sur l'erreur de position, augmentant l'état du système :

```
Az = [A,  0;
     -C,  0]     (5x5)
Bz = [B; D]        (5x1)

z_i_dot = r - C*x     (état intégral, erreur accumulée)
K_lqi = lqr(Az, Bz, Q, R) = [Kx, Ki]
u = -Kx*x - Ki*z_i
```

Le gain complet est ensuite scindé en `Kx` (retour d'état, 1×4) et `Ki` (gain intégral, scalaire) pour reconstruire la boucle fermée avec la référence `r(t)` comme entrée externe :

```
A_cl = [A - B*Kx,   -B*Ki;
        -C,          0]
B_cl = [0; 0; 0; 0; 1]
```

### 3.1 Réglage des matrices de pondération Q et R

Point de départ (règle de Bryson, `Q_ii = 1/x_i,max²`, `R = 1/u_max²`), puis ajustement itératif par balayage de `R` (à `Q` fixe) en observant :
- les pôles dominants et leur amortissement (`damp(Sys_cl)`)
- les performances temporelles (`stepinfo` : temps de montée, dépassement, temps d'établissement)
- le couple maximal demandé (`max(abs(U_cmd))`), à comparer à la limite de 3 N.m

| Jeu de gains | Poles dominants (ζ / ωn) | Rise time | Overshoot | Settling time | Couple max (step) |
|---|---|---|---|---|---|
| `Q = diag([1,5,50,5,500])`, `R = 1` (initial) | ζ=0.585, ωn=2.69 rad/s | 0.878 s | 5.29 % | 2.80 s | 1.27 N.m |
| `Q = diag([1,5,50,5,500])`, `R = 0.05` (retenu) | ζ=0.599, ωn=2.83 rad/s | 0.844 s | 4.91 % | 2.65 s | 1.41 N.m |

Le second réglage améliore légèrement la rapidité et l'amortissement sans coût significatif en couple, et se situe déjà proche de la limite atteignable avec cette architecture (voir §3.4).

### 3.2 Noise Modeling *(non implémenté — piste future / LQG)*

Le système actuel suppose l'état complet `[x, x_dot, theta, theta_dot]` directement mesurable, sans bruit de mesure ni perturbation modélisés. Pour une extension LQG, il faudra définir :
- une matrice de covariance du bruit de process (incertitudes de modèle, perturbations)
- une matrice de covariance du bruit de mesure (bruit capteur — IMU pour `theta`, encodeur pour `x`)

*(À compléter selon les capteurs réellement instrumentés sur le prototype.)*

### 3.3 Observer Dynamics *(non implémenté — piste future / LQG)*

Aucun observateur (filtre de Kalman ou autre) n'a été implémenté à ce stade — la commande actuelle nécessite la mesure directe de tous les états, en particulier `theta` et `theta_dot`, ce qui suppose une IMU ou un capteur d'inclinaison sur le châssis. Si seul `x` (et éventuellement `x_dot`) est mesurable directement, un observateur (Kalman/Luenberger) devient nécessaire pour reconstruire `theta` et `theta_dot` avant de fermer la boucle LQI.

### 3.4 Limites structurelles du système

L'analyse des zéros de transmission de la fonction de transfert `u → x` révèle un **zéro à phase non minimale** en `s = +4.686 rad/s` (`zero(TF_SEGWAY)`). Cette propriété est **invariante par retour d'état** : aucun réglage de `Q`/`R`, même extrême (`R → 0`), ne permet de dépasser durablement une pulsation propre dominante d'environ **60 % de cette valeur (~2.8 rad/s)** sans dégrader fortement la robustesse. Conséquences pratiques :
- Le temps de montée ne peut pas descendre significativement en dessous de ~0.84 s avec cette architecture (retour d'état LQI, sortie `x`).
- Le suivi d'une consigne sinusoïdale à 1 Hz (6.28 rad/s), présente dans le profil de test, est **structurellement hors de portée** de tout retour d'état linéaire sur cette sortie — la fréquence demandée dépasse le zéro instable lui-même.
- Pistes pour dépasser cette limite : préfiltrage / génération de trajectoire réalisable pour la référence, ou reconsidération de la sortie régulée.

## 8. Implementation

- **Simulink** : boucle fermée LQI implémentée et simulée (bloc de gain d'état + intégrateur d'erreur), en parallèle du script d'analyse MATLAB.
- **Script MATLAB** : construction du modèle d'état, analyse en boucle ouverte, synthèse LQR/LQI, simulation de la réponse temporelle avec le profil de référence `Rcmd`, et outils de réglage (balayage `Q`/`R`, `damp`, `stepinfo`, `bandwidth`, `zero`).

*(Détails d'intégration Simulink — schéma-bloc, sous-systèmes, paramétrage des blocs — à compléter.)*

## 9. Conclusion

La loi de commande LQI stabilise le système instable en boucle ouverte et assure un suivi de consigne de position avec de bonnes performances pour un profil de type échelon (dépassement ~5 %, temps d'établissement ~2.7 s, couple requis < 1.5 N.m, sous la limite de 3 N.m retenue). L'analyse des zéros de transmission a mis en évidence une limite structurelle à phase non minimale (~4.7 rad/s) bornant la rapidité atteignable, indépendamment du réglage des gains — un résultat important pour cadrer les attentes de performance du système.

**Travaux futurs** :
- Extension LQG (observateur + modélisation du bruit) si l'état complet n'est pas directement mesurable
- Préfiltrage de la consigne pour les profils à haute fréquence (au-delà de la bande passante atteignable)
- Intégration dans un scénario de visualisation 3D
- Validation expérimentale sur prototype et confirmation de la limite de couple moteur réelle
