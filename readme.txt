VERTEX_TRACKXYZ id, x, y, z
    La mappa nella quale localizzarsi e' espressa come collezione di Landmark, la cui posizione nello spazio va considerata ground truth. E' espressa nel sistema di riferimento origine(mondo)


    VERTEX_SE3:QUAT id, x, y, z, qx, qy, qz, qw
    Il dataset fornisce anche l'odometria della piattaforma su cui e' montato il sensore(la camera). I Vertex sono espressi anch'essi nel sistema di riferimento origine(mondo), e rappresentano una misura soggetta a rumore.

    EDGE_SE3:QUAT pose_id_start pose_id_goal x y z qx qy qz qw  information_matrix
    Gli edge sono le trasformate relative tra una posa(un vertex) e la successiva. Forniscono informazione sul rumore a cui e' soggetta l'odometria tramite una matrice di informazione(errata corrige, nel vecchio Readme avevo scritto covarianza). La matrice si ricostruisce come segue (blocco 3x3 per la traslazione, blocco 3x3 per la rotazione, i.e. solo qx qy qz poiche' il quaternione e' considerato come unitario)

100 0     0     0         0          0 

       100 0     0         0          0 

              100 0         0          0 

                     10000 0          0 

                                10000  0 

                                           10000

    EDGE_PROJECT_DEPTH pose_id landmark_id sensor_id u v d information_matrix
    Un landmark viene osservato da una posa, con un certo sensore, nel pixel [u,v] con depth d e una certa information matrix, ricostruita come segue

1000 0       0 

         1000 0 

                  1000



Cose mancanti nel README

PARAMS_CAMERACALIB sensor_id x y z qx qy qz qw fx fy cx cy

Il dataset fornisce, per ogni sensore(una singola camera in questo caso) la sua posa relativa alla piattaforma su cui e' montato. Inoltre fornisce le componenti della camera matrix (nel dataset in questione la camera matrix e' ideale, infatti le osservazioni vengono fatte in pixel compresi tra [0,1])



Per altre info, resto a disposizione

Bartolomeo Della Corte