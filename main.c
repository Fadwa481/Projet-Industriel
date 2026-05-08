#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <float.h>

/*  STRUCTURE DE DONNÉES
    Topologie hétérogène imposée par le cahier des charges */

struct Drone {
    int   id;   /* Identifiant unique du drone */
    float x;    /* Coordonnée spatiale X  */
    float y;    /* Coordonnée spatiale Y  */
    float z;    /* Coordonnée spatiale Z (altitude) */
};

/*  CONSTANTES SYSTÈME */

#define N_DRONES    10000   /* Taille de l'essaim */
#define ESPACE_MAX  10000.0f /* Volume cubique de vol */

/*  CALCUL DE DISTANCE EUCLIDIENNE 3D */
float distance_euclidienne(const struct Drone *a, const struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* Distance au carré  */
float distance_carre(const struct Drone *a, const struct Drone *b) {
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return dx*dx + dy*dy + dz*dz;
}

/* COMPARATEUR POUR qsort  TRI PAR AXE X */
int comparateur_axe_x(const void *ptr_a, const void *ptr_b) {
    /* Conversion des pointeurs génériques vers le type Drone */
    const struct Drone *drone_a = (const struct Drone *)ptr_a;
    const struct Drone *drone_b = (const struct Drone *)ptr_b;

    /* Comparaison sur l'axe X */
    if (drone_a->x < drone_b->x) return -1;
    if (drone_a->x > drone_b->x) return  1;
    return 0;
}

/*  INITIALISATION DE L'ESSAIM */
void initialiser_essaim(struct Drone *essaim, int n) {
    int i;
    struct Drone *ptr_courant; /* Pointeur navigateur */

    for (i = 0; i < n; i++) {
        /* Arithmétique de pointeurs : accès à l'élément i */
        ptr_courant = essaim + i;

        ptr_courant->id = i;
        ptr_courant->x  = ((float)rand() / RAND_MAX) * ESPACE_MAX;
        ptr_courant->y  = ((float)rand() / RAND_MAX) * ESPACE_MAX;
        ptr_courant->z  = ((float)rand() / RAND_MAX) * ESPACE_MAX;
    }
}

/*  RECHERCHE DE LA PAIRE LA PLUS PROCHE — FENÊTRE GLISSANTE */
float rechercher_paire_minimale(struct Drone *essaim, int n,
                                 int *idx_a, int *idx_b) {
    int i, j;
    float dist_min = FLT_MAX;   /* Distance minimale courante */
    float dist_courante;
    int   fenetre = 20;         /* Taille de la fenêtre de voisinage */

    const struct Drone *drone_i; /* Pointeur vers le drone i */
    const struct Drone *drone_j; /* Pointeur vers le drone j (voisin) */

    *idx_a = 0;
    *idx_b = 1;

    for (i = 0; i < n - 1; i++) {
        /* Pointeur arithmétique vers le drone i */
        drone_i = essaim + i;

        /* Comparaison avec les 'fenetre' voisins suivants */
        for (j = i + 1; j < n && j <= i + fenetre; j++) {
            /* Pointeur arithmétique vers le drone j */
            drone_j = essaim + j;

            dist_courante = distance_carre(drone_i, drone_j);

            if (dist_courante < dist_min) {
                dist_min = dist_courante;
                *idx_a   = i;
                *idx_b   = j;
            }
        }
    }

    /* On retourne la vraie distance (avec sqrt) uniquement pour l'affichage */
    return sqrtf(dist_min);
}

/* AFFICHAGE D'UN DRONE */
void afficher_drone(const struct Drone *d) {
    printf("  Drone #%5d | X=%9.3f | Y=%9.3f | Z=%9.3f\n",
           d->id, d->x, d->y, d->z);
}

/*  PROGRAMME PRINCIPAL */
int main(void) {
    struct Drone *essaim = NULL; /* Pointeur vers l'entrepôt mémoire */
    int idx_a, idx_b;            /* Indices de la paire la plus proche */
    float distance_min;
    clock_t debut, fin;
    double temps_execution;

    
    printf("  SYSTÈME DE DÉTECTION DE COLLISION - ESSAIM UAV\n");
    printf("  N = %d drones | Algorithme O(n log n)\n", N_DRONES);
  

    /* ÉTAPE 1 : Allocation dynamique du tableau 
      malloc alloue un bloc CONTINU de N_DRONES structures Drone.
      "essaim" pointe sur le PREMIER drone de ce bloc.
      Toute navigation se fera par arithmétique de pointeurs.
     */
    essaim = (struct Drone *)malloc(N_DRONES * sizeof(struct Drone));
    if (essaim == NULL) {
        fprintf(stderr, "ERREUR CRITIQUE : Allocation mémoire échouée.\n");
        return EXIT_FAILURE;
    }
    printf("[1/4] Allocation mémoire : %lu octets alloués.\n",
           (unsigned long)(N_DRONES * sizeof(struct Drone)));

    /*  ÉTAPE 2 : Initialisation de l'essaim  */
    srand((unsigned int)time(NULL));
    initialiser_essaim(essaim, N_DRONES);
    printf("[2/4] Essaim initialisé avec %d drones.\n", N_DRONES);

    /* ÉTAPE 3 : Tri par axe X — O(n log n) 
      qsort utilise le comparateur sur l'axe X.
      Après tri, les drones potentiellement proches sont adjacents.
     */
    debut = clock();
    qsort(essaim,
          N_DRONES,
          sizeof(struct Drone),
          comparateur_axe_x);
    printf("[3/4] Tri par axe X terminé (qsort, O(n log n)).\n");

    /* ÉTAPE 4 : Recherche de la paire minimale O(n)  */
    distance_min = rechercher_paire_minimale(essaim, N_DRONES,
                                             &idx_a, &idx_b);
    fin = clock();
    temps_execution = (double)(fin - debut) / CLOCKS_PER_SEC * 1000.0;

    printf("[4/4] Recherche terminée.\n\n");

    /*  RÉSULTAT  */
    printf("  RÉSULTAT : PAIRE DE DRONES LA PLUS PROCHE\n");
    afficher_drone(essaim + idx_a);   /* Arithmétique de pointeurs */
    afficher_drone(essaim + idx_b);   /* Arithmétique de pointeurs */
    printf("\n  Distance minimale : %.4f mètres\n", distance_min);
    printf("  Temps d'exécution (tri + recherche) : %.3f ms\n",
           temps_execution);
   
    /*  Libération mémoire  */
    free(essaim);
    essaim = NULL;

    return EXIT_SUCCESS;
}
