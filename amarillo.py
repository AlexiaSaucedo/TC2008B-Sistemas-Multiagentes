import agentpy as ap
import numpy as np
import matplotlib.pyplot as plt
import copy

"""""""""""""""""""""""""""""""""APERTURA DE ARCHIVOS"""""""""""""""""""""""""""""""""""""""""""""""""""
#myFile = open("resultados.csv", "w")
#myFile.write("Poblacion,Destinos alcanzados,Tiempo promedio,Tiempo primer destino,Tiempo ultimo destino,Choques\n")
fileInicio = open("parametros.csv", "w")
fileCarros = open("infoCarros.csv", "w")
fileSemaforos = open("infoSemaforos.csv" ,"w")

"""""""""""""""""""""""""""""""""GRAFO"""""""""""""""""""""""""""""""""""""""""""""""""""
def ndarrayGrafo():
    pGrafo = np.ndarray(shape=(2,len(grafo)), dtype=float, order='F')
    for i in range(len(grafo)):
        pGrafo[0][i] = grafo[i][0]
        pGrafo[1][i] = grafo[i][1]
    return pGrafo

#Coordenadas de cada nodo
grafo = [[42.0,2.0,0.0],[42.0,29.0,0.0],[42.0,32.0,0.0],[42.0,33.0,0.0],
         [2.0,38.0,0.0],[29.0,38.0,0.0],[32.0,38.0,0.0],[33.0,38.0,0.0],
         [38.0,78.0,0.0],[38.0,51.0,0.0],[38.0,48.0,0.0],[38.0,47.0,0.0],
         [78.0,42.0,0.0],[51.0,42.0,0.0],[48.0,42.0,0.0],[47.0,42.0,0.0],
         
         [38.0,32.0,0.0],[38.0,2.0,0.0],
         [32.0,42.0,0.0],[2.0,42.0,0.0],
         [42.0,48.0,0.0],[42.0,78.0,0.0],
         [48.0,38.0,0.0],[78.0,38.0,0.0]]

nodosIniciales=[0,4,8,12]
nodosFinales=[17,19,21,23]
listaAdyacencias = [[1],[2],[3],[16,18,20,22],
                     [5],[6],[7],[16,18,20,22],
                     [9],[10],[11],[16,18,20,22],
                     [13],[14],[15],[16,18,20,22],
                     
                     [17],[],
                     [19],[],
                     [21],[],
                     [23],[]]

#Lista de todos los nodos de cada carril
nodosCarriles = [[0,1,2,3],[4,5,6,7],[8,9,10,11],[12,13,14,15],[16,17],[18,19],[20,21],[22,23]]

#Lista de flujo entre carriles, si este se referencia a sí mismo significa que es un carril con nodo final
conexionCarriles = [[4,5,6,7],[4,5,6,7],[4,5,6,7],[4,5,6,7],[4],[5],[6],[7]]

posGrafo = ndarrayGrafo()

#Función para encontrar el carril al que pertenece un nodo dado
def findNodeInCamino(n):
    for i in range(len(nodosCarriles)):
        for j in nodosCarriles[i]:
            if j == n:
                return i
    return -1

#Función para crear un trayecto al azar por medio de los carriles
def createRandomCaminoPath(inicial, visitados):
    nextArr = []
    nextArr.append(inicial)
    visitados[inicial] = True
    caminoArr = copy.deepcopy(conexionCarriles[inicial])
    np.random.shuffle(caminoArr)
    for i in range(len(caminoArr)):
        if(caminoArr[i] != inicial and not visitados[caminoArr[i]]):
            nextArr = nextArr + createRandomCaminoPath(caminoArr[i], visitados)
            if(len(nextArr)>1):
                return nextArr
        elif(caminoArr[i] == inicial):
            return nextArr
    return []

#Función para crear un trayecto al azar por medio de los nodos tomando en cuenta un trayecto basado en carriles
def createPathFromCamino(caminos, ind,indi):
    pathN = []
    pathC = []
    listNodos = copy.deepcopy(nodosCarriles[caminos[ind]][indi:])
    np.random.shuffle(listNodos)
    if(ind < len(caminos)-1):
        for i in range(len(listNodos)):
            for j in range(len(listaAdyacencias[listNodos[i]])):
                if(findNodeInCamino(listaAdyacencias[listNodos[i]][j]) == caminos[ind +1]):    
                    auxPathN, auxPathC = createPathFromCamino(caminos, ind+1, nodosCarriles[caminos[ind+1]].index(listaAdyacencias[listNodos[i]][j]))
                    pathN = nodosCarriles[caminos[ind]][indi:nodosCarriles[caminos[ind]].index(listNodos[i])+1] + auxPathN
                    pathC = [caminos[ind]]*len(nodosCarriles[caminos[ind]][indi:nodosCarriles[caminos[ind]].index(listNodos[i])+1])
                    pathC = pathC + auxPathC
                    return pathN, pathC
    else:
        return nodosCarriles[caminos[ind]][indi:], [caminos[ind]]*len(listNodos)
    return pathN, pathC

"""""""""""""""""""""""""""""""""SEMAFORO"""""""""""""""""""""""""""""""""""""""""""""""""""
#Estado de los semáforos, 0 es verde, 1 es rojo
semaforoEstado = [0,1,1,1]
#Semáforo que se le asigna a cada nodo; -1 significa que no tiene
semaforoNodo = [-1,0,0,-1,
                -1,1,1,-1,
                -1,2,2,-1,
                -1,3,3,-1,
                -1,-1,-1,-1
                -1,-1,-1,-1]

#Carriles a los que influye cada semáforo
semaforoCaminos = [[0],[1],[2],[3]]
#Duración del tiempo en verde de cada semáforo
semaforoTiempoVerde = [3.0,3.0,3.0,3.0]
#Posición de los semáforos en la simulación
semaforoPosicion = [[42.0,33.0,6.0],[33.0,38.0,6.0],[38.0,47.0,6.0],[47.0,42.0,6.0]]

"""""""""""""""""""""""""""""""""AGENTE SEMAFORO"""""""""""""""""""""""""""""""""""""""""""""""""""
class SemaforoAgent(ap.Agent):
    def setup(self):
        self.estado = 0
        self.tiempoCambios = 0.0
        self.cronometro = 0.0
        self.cantCarros = 0
        self.turnosEspera = 0
        
    def initSetup(self, ind):
        self.estado = semaforoEstado[ind]
        self.caminos = semaforoCaminos[ind]
        self.tiempoCambios = semaforoTiempoVerde[ind]
        self.currentSem = semaforoEstado.index(0)
        self.nextSem = self.currentSem
    
    #Función para obtener la cantidad de carros en sus carriles
    def contarCarros(self, carrosAg):
        self.cantCarros = 0
        for i in carrosAg:
            if i.currentCamino in self.caminos:
                self.cantCarros += 1
    
    #Función para actualizar el cronómetro del semáforo en verde
    def actualizarTiempo(self, ti):
        fileSemaforos.write(str(self.estado)+"\n")
        if self.estado != 1:
            self.cronometro += ti
    

"""""""""""""""""""""""""""""""""AGENTE CARRO"""""""""""""""""""""""""""""""""""""""""""""""""""
class CarroAgent(ap.Agent):

    def setup(self):
        self.maxVelocidad = float(np.random.randint(11,14))
        self.minVelocidad = 0.0
        self.maxAceleracion = 10.0
        self.minAceleracion = -40.0
        self.aceleracion = 0.0
        self.velocidad = self.maxVelocidad
        self.direccionVelocidad = [0,0]
        self.rotacionCarro = 0.0
        self.maxRotacionLlantas = 20.0
        self.finished = False
        self.choque = False
        self.tiempoLlegada = 0.0
        self.maxDistanciaVecino = 5.0
        self.maxAnguloVecino = 15.0
        
              
    def setup_pos(self, space, contadorCarros):
        self.neighbors = space.neighbors
        self.neighborList = self.neighbors(self,distance=120).to_list() 
        self.space = space
        self.pos = space.positions[self]
        self.currentNode = 0
        self.currentPathI = 0
        self.checarNodoInicial()
        self.nodePath, self.caminoPath = createPathFromCamino(createRandomCaminoPath(findNodeInCamino(self.currentNode), [False]*len(nodosCarriles)), 0,0)
        self.currentCamino = self.caminoPath[0]
        self.finalNode = self.nodePath[-1]
        
        self.nextNode = self.nodePath[self.currentPathI+1]
        self.distanciaNext = 0.0
        self.anguloNext = 0.0
        self.rotacionLlantas = 0.0
        
        self.initNextDistancia = 0.0
        self.initZpos = self.pos[2]
        self.initDiferenciaAltura = grafo[self.nextNode][2] - self.pos[2]
        self.zAngle = 0.0
        if(self.initNextDistancia != 0.0):
            self.zAngle = np.rad2deg(np.arctan(self.initDiferenciaAltura/self.initNextDistancia))
        
        self.inicializarRotacionCarro()
        self.pos[0] -= (self.direccionVelocidad[0] * contadorCarros[nodosIniciales.index(self.currentNode)])
        self.pos[1] -= (self.direccionVelocidad[1] * contadorCarros[nodosIniciales.index(self.currentNode)])
        contadorCarros[nodosIniciales.index(self.currentNode)]+=1;

    #Función para calcular la distancia entre el carro y su siguiente nodo y el ángulo desde el carro y el siguiente nodo
    def calcularRelacionNextNode(self):
        diferenciaY = grafo[self.nextNode][1] - self.pos[1]
        diferenciaX = grafo[self.nextNode][0] - self.pos[0]
        self.distanciaNext=np.sqrt(np.power(diferenciaY,2)+np.power(diferenciaX,2))
        if(diferenciaX==0):
            if(diferenciaY>0):
                self.anguloNext = 90
            else:
                self.anguloNext = 270
        elif(diferenciaY==0):
            if(diferenciaX>0):
                self.anguloNext = 0
            else:
                self.anguloNext = 180
        else:
            self.anguloNext = np.rad2deg(np.arctan(diferenciaY/diferenciaX))
            if(self.anguloNext>0):
                if(diferenciaX<0):
                    self.anguloNext = 180 + self.anguloNext
            else:
                if(diferenciaX<0):
                    self.anguloNext = 180 + self.anguloNext
                else:
                    self.anguloNext = 360 + self.anguloNext
    
    #Función para encontrar el índice del nodo inicial del carro dada su posición
    def checarNodoInicial(self):
        for i in range(len(grafo)):
            if grafo[i][0] == self.pos[0] and grafo[i][1] == self.pos[1]:
                self.currentNode = i
    
    #Función para calcular la rotación inicial del carro
    def inicializarRotacionCarro(self):
        diferenciaY = grafo[self.nextNode][1] - grafo[self.currentNode][1]
        diferenciaX = grafo[self.nextNode][0] - grafo[self.currentNode][0]
        self.distanciaNext=np.sqrt(np.power(diferenciaY,2)+np.power(diferenciaX,2))
        self.initNextDistancia =self.distanciaNext
        
        if(diferenciaX==0):
            if(diferenciaY>0):
                self.rotacionCarro = 90
            else:
                self.rotacionCarro = 270
        elif(diferenciaY==0):
            if(diferenciaX>0):
                self.rotacionCarro = 0
            else:
                self.rotacionCarro = 180
        else:
            self.rotacionCarro = np.rad2deg(np.arctan(diferenciaY/diferenciaX))
            if(self.rotacionCarro>0):
                if(diferenciaX<0):
                    self.rotacionCarro = 180 + self.rotacionCarro
            else:
                if(diferenciaX<0):
                    self.rotacionCarro = 180 + self.rotacionCarro
                else:
                    self.rotacionCarro = 360 + self.rotacionCarro
        self.direccionVelocidad[0] = np.cos(np.deg2rad(self.rotacionCarro + self.rotacionLlantas)) * self.velocidad
        self.direccionVelocidad[1] = np.sin(np.deg2rad(self.rotacionCarro + self.rotacionLlantas)) * self.velocidad
        self.anguloNext = self.rotacionCarro
    
    #Función para calcular la rotación de las llantas dada un ángulo
    def formulaRotacionLlanta(self, vx):
        return self.maxRotacionLlantas/(1+ np.exp((-1/40)*(vx-90)))
    
    #Función para calcular la rotación del carro dada su vector de velocidad
    def calcularRotacionCarro(self):
        if(self.velocidad > 0):
            if(self.direccionVelocidad[0]==0):
                if(self.direccionVelocidad[1]>0):
                    self.rotacionCarro = 90
                else:
                    self.rotacionCarro = 270
            elif(self.direccionVelocidad[1]==0):
                if(self.direccionVelocidad[0]>0):
                    self.rotacionCarro = 0
                else:
                    self.rotacionCarro = 180
            else:
                self.rotacionCarro = np.rad2deg(np.arctan(self.direccionVelocidad[1]/self.direccionVelocidad[0]))
                if(self.rotacionCarro>0):
                    if(self.direccionVelocidad[0]<0):
                        self.rotacionCarro = 180 + self.rotacionCarro
                else:
                    if(self.direccionVelocidad[0]<0):
                        self.rotacionCarro = 180 + self.rotacionCarro
                    else:
                        self.rotacionCarro = 360 + self.rotacionCarro
    
    #Función para calcular la rotación de las llantas con base a la posición del siguiente nodo
    def calcularRotacionLlantas(self):
        if(self.rotacionCarro == self.anguloNext):
            self.rotacionLlantas = 0.0
        elif(self.rotacionCarro < self.anguloNext):
            if(np.abs(self.anguloNext-self.rotacionCarro) < np.abs(self.anguloNext-self.rotacionCarro-360)):
                if(np.abs(self.anguloNext-self.rotacionCarro) > 60 and self.distanciaNext < 3):
                    self.aceleracion = self.minAceleracion/2
                self.rotacionLlantas = self.formulaRotacionLlanta(np.abs(self.anguloNext-self.rotacionCarro))
                
            else:
                if(np.abs(self.rotacionCarro-self.anguloNext-360) > 60 and self.distanciaNext < 3):
                    self.aceleracion = self.minAceleracion/2
                self.rotacionLlantas = -1*self.formulaRotacionLlanta(np.abs(self.anguloNext-self.rotacionCarro-360))
                
        else:
            if(np.abs(self.rotacionCarro-self.anguloNext) < np.abs(self.rotacionCarro-self.anguloNext-360)):
                if(np.abs(self.anguloNext-self.rotacionCarro) > 60 and self.distanciaNext < 3):
                    self.aceleracion = self.minAceleracion/2
                self.rotacionLlantas = -1*self.formulaRotacionLlanta(np.abs(self.rotacionCarro-self.anguloNext))
                
            else:
                if(np.abs(self.rotacionCarro-self.anguloNext-360) > 60 and self.distanciaNext < 3):
                    self.aceleracion = self.minAceleracion/2
                self.rotacionLlantas = self.formulaRotacionLlanta(np.abs(self.rotacionCarro-self.anguloNext-360))
    
    #Función para checar si el carro llegó al siguiente nodo
    def checarNextNode(self, tiempo):
        if(np.sqrt(np.power(grafo[self.nextNode][1] - self.pos[1],2)+np.power(grafo[self.nextNode][0] - self.pos[0],2)) < 1):
            self.currentNode = self.nextNode
            self.currentPathI += 1
            if(self.currentPathI < len(self.nodePath)-1):
                self.nextNode = self.nodePath[self.currentPathI+1]
                self.currentCamino = self.caminoPath[self.currentPathI]
                diferenciaY = grafo[self.nextNode][1] - grafo[self.currentNode][1]
                diferenciaX = grafo[self.nextNode][0] - grafo[self.currentNode][0]
                self.distanciaNext=np.sqrt(np.power(diferenciaY,2)+np.power(diferenciaX,2))
                self.initNextDistancia =self.distanciaNext
                self.initZpos = self.pos[2]
                self.initDiferenciaAltura = grafo[self.nextNode][2] - self.pos[2]
                self.zAngle = np.rad2deg(np.arctan(self.initDiferenciaAltura/self.initNextDistancia))
            else:
                self.velocidad = 0.0
                self.finished=True
                self.tiempoLlegada = tiempo
                self.pos[0]=100.0
                self.pos[1]=100.0
                
    #Función para calcular la distancia entre el carro y sus vecinos y el ángulo desde el carro y sus vecinos
    def calcularRelacionVecinos(self):
        distancia=self.maxDistanciaVecino
        anguloMaximo=15.0
        if(len(self.neighborList)>0):
            for nbs in self.neighborList:
                if(np.sqrt(np.power(nbs.pos[1] - self.pos[1],2)+np.power(nbs.pos[0] - self.pos[0],2)+np.power(nbs.pos[2] - self.pos[2],2)) <= distancia):
                    distancia = np.sqrt(np.power(nbs.pos[1] - self.pos[1],2)+np.power(nbs.pos[0] - self.pos[0],2)+np.power(nbs.pos[2] - self.pos[2],2))
                    if(distancia<=0.3 and not self.finished):
                        self.finished = True
                        self.choque = True
                        self.velocidad = 0.0
                        
                    diferenciaY = nbs.pos[1] - self.pos[1]
                    diferenciaX = nbs.pos[0] - self.pos[0]
                    if(diferenciaX==0):
                        if(diferenciaY>0):
                            anguloVecino = 90
                        else:
                            anguloVecino = 270
                    elif(diferenciaY==0):
                        if(diferenciaX>0):
                            anguloVecino = 0
                        else:
                            anguloVecino = 180
                    else:
                        anguloVecino = np.rad2deg(np.arctan(diferenciaY/diferenciaX))
                        if(anguloVecino>0):
                            if(diferenciaX<0):
                                anguloVecino = 180 + anguloVecino
                        else:
                            if(diferenciaX<0):
                                anguloVecino = 180 + anguloVecino
                            else:
                                anguloVecino = 360 + anguloVecino
                    
                    if(self.rotacionCarro == anguloVecino):
                        self.aceleracion = self.minAceleracion
                        
                    elif(self.rotacionCarro < anguloVecino):
                        if(np.abs(anguloVecino-self.rotacionCarro) < np.abs(anguloVecino-self.rotacionCarro-360)):
                            if(np.abs(anguloVecino-self.rotacionCarro) <= self.maxAnguloVecino):
                                self.aceleracion = self.minAceleracion
                        else:
                            if(np.abs(self.rotacionCarro-anguloVecino-360) <= self.maxAnguloVecino):
                                self.aceleracion = self.minAceleracion

                    else:
                        if(np.abs(self.rotacionCarro-anguloVecino) < np.abs(self.rotacionCarro-anguloVecino-360)):
                            if(np.abs(anguloVecino-self.rotacionCarro) <= self.maxAnguloVecino):
                                self.aceleracion = self.minAceleracion
                        
                        else:
                            if(np.abs(self.rotacionCarro-anguloVecino-360) <= self.maxAnguloVecino):
                                self.aceleracion = self.minAceleracion
                    
    #Función para actualizar la aceleración, velocidad y posición del carro             
    def movimiento(self, tiempo, agentSem):
        fileCarros.write(str(self.pos[0])+","+str(self.pos[1])+","+str(self.pos[2])+","+str(self.rotacionCarro)+","+str(self.zAngle)+"\n") 
        if(not self.finished):
            
            self.aceleracion = self.maxAceleracion
            #Calculo del ángulo del auto
            self.calcularRotacionCarro()
            
            #Cálculo de la rotación de las llantas y frenado para vueltas
            self.calcularRelacionNextNode()
            self.calcularRotacionLlantas()
            
            #Frenado por semáforo
            if(semaforoNodo[self.currentNode]!=-1):
                if(agentSem[semaforoNodo[self.currentNode]].estado!=0):
                    self.aceleracion = self.minAceleracion
            
            self.calcularRelacionVecinos()
            
            #Cálculo de velocidad y desplazamiento
            self.velocidad += (self.aceleracion*self.p['second_per_step'])
            if(self.velocidad < self.minVelocidad):
                self.velocidad = self.minVelocidad
            if(self.velocidad > self.maxVelocidad):
                self.velocidad = self.maxVelocidad
            
            self.direccionVelocidad[0] = np.cos(np.deg2rad(self.rotacionCarro + self.rotacionLlantas)) * self.velocidad
            self.direccionVelocidad[1] = np.sin(np.deg2rad(self.rotacionCarro + self.rotacionLlantas)) * self.velocidad
            
            self.pos[0] += (self.direccionVelocidad[0] * self.p['second_per_step'])
            self.pos[1] += (self.direccionVelocidad[1] * self.p['second_per_step'])
            
            diferenciaY = grafo[self.nextNode][1] - self.pos[1]
            diferenciaX = grafo[self.nextNode][0] - self.pos[0]
            self.pos[2] = self.initZpos + ((self.initNextDistancia - (np.sqrt(np.power(diferenciaY,2)+np.power(diferenciaX,2)))) * (self.initDiferenciaAltura / self.initNextDistancia))
            
            #Se checa si cambió de nodo
            self.checarNextNode(tiempo)
            pass

"""""""""""""""""""""""""""""""""MODELO"""""""""""""""""""""""""""""""""""""""""""""""""""
class MyModel(ap.Model):

    def setup(self):
        self.contadorCarros=[0]*len(nodosIniciales)
        self.tiempoPrimeraLlegada = 0.0
        self.tiempoUltimaLlegada = 0.0
        self.tiemposPromedioLlegada = 0.0
        self.destinos = 0
        self.choques = 0
        
        self.space = ap.Space(self, shape=[self.p.size]*self.p.ndim)
        self.agents = ap.AgentList(self, self.p.population, CarroAgent)
        self.space.add_agents(self.agents, positions=self.initPosicion())
        self.agents.setup_pos(self.space, self.contadorCarros)
        self.agentSem = ap.AgentList(self, len(semaforoEstado), SemaforoAgent)
        
        for i in range(len(self.agentSem)):
            self.agentSem[i].initSetup(i)
            self.agentSem[i].contarCarros(self.agents)
        
        

    def step(self):
        self.agentSem.actualizarTiempo(self.p['second_per_step'])
        self.agentSem.contarCarros(self.agents)
        self.administrarSemaforos()
        self.agents.movimiento(self.t*self.p['second_per_step'], self.agentSem)
        plt.show()

    def update(self):
        """ Record a dynamic variable. """
        

    def end(self):
        """ Repord an evaluation measure. """
        #self.calcularResultados()
        #self.escribirResultados()
        #self.imprimirResultados()
        #self.record(['tiempoPrimeraLlegada', 'tiempoUltimaLlegada', 'tiemposPromedioLlegada', 'destinos', 'choques'])
        
    #Función para asignar al azar una posición inicial
    def initPosicion(self):
        res = []
        for i in range(self.p.population):
            res.append(grafo[nodosIniciales[np.random.randint(0,len(nodosIniciales))]])
        return res
    
    #Función para encontrar la cantidad total de autos que están en carriles con semáforo
    def contarTotalCarros(self):
        num = 0
        for i in self.agentSem:
            num += i.cantCarros
        return num
    
    #Función para realizar los cambios de estado de los semáforos en base a la cantidad de autos y los turnos de espera de cada uno
    def administrarSemaforos(self):
        currentSemaforo = self.agentSem[0].currentSem
        indexCambio = self.agentSem[0].nextSem
        
        if(self.agentSem[currentSemaforo].cronometro >= 1.0 and self.agentSem[currentSemaforo].estado == 2):
            self.agentSem[currentSemaforo].cronometro = 0.0
            self.agentSem[currentSemaforo].estado = 1
            self.agentSem[indexCambio].estado = 0
            self.agentSem[indexCambio].turnosEspera = 0
            
            for i in range(len(self.agentSem)):
                if(i != currentSemaforo and i != indexCambio):
                    self.agentSem[i].turnosEspera += 1
                self.agentSem[i].currentSem = indexCambio
        
        elif(self.agentSem[currentSemaforo].cronometro >= self.agentSem[currentSemaforo].tiempoCambios or (self.agentSem[currentSemaforo].cantCarros == 0 and self.contarTotalCarros() > 0)):
            maxCarros = 0
            maxCarrosI = currentSemaforo
            maxTurnos = 0
            maxTurnosI = 0
            for i in range(len(self.agentSem)):
                if(self.agentSem[i].cantCarros > maxCarros):
                    maxCarros = self.agentSem[i].cantCarros
                    maxCarrosI = i
                if(self.agentSem[i].turnosEspera > len(self.agentSem) and self.agentSem[i].turnosEspera > maxTurnos):
                    maxTurnos = self.agentSem[i].turnosEspera
                    maxTurnosI = i        
            
            if(maxTurnos > 0):
                indexCambio = maxTurnosI
            else:
                indexCambio = maxCarrosI
            
            self.agentSem[currentSemaforo].cronometro = 0.0
            self.agentSem[currentSemaforo].estado = 2
            
            for i in range(len(self.agentSem)):
                self.agentSem[i].nextSem = indexCambio
                
    #Función para calcular los resultados finales de la simulación
    def calcularResultados(self):
        tiempos = []
        for ag in self.agents:
            if(ag.tiempoLlegada > 0.0):
                tiempos.append(ag.tiempoLlegada)
            if(ag.choque):
                self.choques += 1
            elif(ag.finished):
                self.destinos += 1
        if(len(tiempos)>0):
            self.tiempoPrimeraLlegada = min(tiempos)
            self.tiempoUltimaLlegada = max(tiempos)
            self.tiemposPromedioLlegada = sum(tiempos)/self.destinos
    
    #Función para escribir en el archivo los resultados de la simulación
    def escribirResultados(self):
        global myFile
        
        writeContent=""
        writeContent += (str(self.p['population']) + ",")
        writeContent += (str(self.destinos) + ",")
        writeContent += (str(self.tiemposPromedioLlegada) + ",")
        writeContent += (str(self.tiempoPrimeraLlegada) + ",")
        writeContent += (str(self.tiempoUltimaLlegada) + ",")
        writeContent += (str(self.choques) + "\n")
        myFile.write(writeContent)
    
    #Función para imprimir los resultados en consola
    def imprimirResultados(self):
        print()
        print("Poblacion: " + str(self.p['population']))
        print("Destinos alcanzados: " + str(self.destinos) + "/" +str(self.p['population']))
        print("Tiempo promedio de destinos: " + str(self.tiemposPromedioLlegada))
        print("Tiempo primer destino alcanzado: " + str(self.tiempoPrimeraLlegada))
        print("Tiempo ultimo destino alcanzado: " + str(self.tiempoUltimaLlegada))
        print("Choques: " + str(self.choques) + "/" +str(self.p['population']))

"""""""""""""""""""""""""""""""""ANIMACION"""""""""""""""""""""""""""""""""""""""""""""""""""
parameters2D = {
    'size': 80,
    'steps': 2500,
    'ndim': 3,
    'population': 20,
    'second_per_step': 0.025
}

def animation_plot_single(m, ax):
    ndim = m.p.ndim
    ax.set_title(f"Movimiento automóvil {ndim}D t={m.t}")
    pos = m.space.positions.values()
    pos = np.array(list(pos)).T  # Transform
    ax.scatter(*pos, s=50, c='black')
    ax.scatter(*posGrafo, s=50, c='red')
    ax.set_xlim(0, m.p.size)
    ax.set_ylim(0, m.p.size)
    if ndim == 3:
        ax.set_zlim(0, m.p.size)
    ax.set_axis_off()

def animation_plot(m, p):
    projection = '3d' if p['ndim'] == 3 else None
    fig = plt.figure(figsize=(7,7))
    ax = fig.add_subplot(111, projection=projection)
    animation = ap.animate(m(p), fig, ax, animation_plot_single)
    animation.save('animacionPrueba.gif', writer='imagemagik', fps=1/p['second_per_step'])


"""""""""""""""""""""""""""""""""CORRIDA"""""""""""""""""""""""""""""""""""""""""""""""""""

#animation_plot(MyModel, parameters2D)

experimentos=1
sample = ap.Sample(parameters2D)
exp = ap.Experiment(MyModel, sample, iterations=experimentos)
results=exp.run()

fileInicio.write(str(parameters2D['population'])+","+str(len(semaforoEstado))+","+str(parameters2D['steps'])+","+str(parameters2D['second_per_step'])+"\n")
for i in semaforoPosicion:
    fileInicio.write(str(i[0])+","+str(i[1])+","+str(i[2])+"\n")
fileInicio.close()
fileCarros.close() 
fileSemaforos.close() 

#myFile.close()