import agentpy as ap
import numpy as np
import matplotlib.pyplot as plt

tiempo = [0.0]
tiemposLlegada = []
choques = [0]
totalTiempoPrimera = []
totalTiempoUltima=[]
totalChoques=[]
totalDestinos=[]
totalTiempoPromedio=[]

"""""""""""""""""""""""""""""""""GRAFO"""""""""""""""""""""""""""""""""""""""""""""""""""
def ndarrayGrafo():
    pGrafo = np.ndarray(shape=(2,len(grafo)), dtype=float, order='F')
    for i in range(len(grafo)):
        pGrafo[0][i] = grafo[i][0]
        pGrafo[1][i] = grafo[i][1]
    return pGrafo


grafo = [[20.0,2.0],[20.0,38.0],
         [2.0,20.0],[38.0,20.0],
         [20.0,15.0], [20.0,25.0],
         [15.0,20.0],[25.0,20.0],
         [20.0,12.0],[12.0,20.0],
         [20.0,17.0], [22.0,19.0],
         [17.0,20.0], [19.0,22.0]]
nodosIniciales=[0,2]
nodosFinales=[1,3]
listaAdyacencias = [[8],[],
                    [9],[],
                    [10,11],[1],
                    [12,13],[3],
                    [4],[6],
                    [5],[7],
                    [7],[5]]
contadorCarros=[0,0]
posGrafo = ndarrayGrafo()

"""""""""""""""""""""""""""""""""SEMAFORO"""""""""""""""""""""""""""""""""""""""""""""""""""
semaforoEstado = [0,1]
semaforoNodo = [-1,-1,-1,-1,0,-1,1,-1,0,1,-1,-1,-1,-1]
#Primero tiempo de duración de verde, luego tiempo duración de rojo
semaforoTimers = [[5.0,5.0],[5.0,5.0]]
semaforoTiempos = [0.0,0.0]

def actualizarSemaforos(ti):
    for i in range(len(semaforoEstado)):
        semaforoTiempos[i] += ti
        if(semaforoTiempos[i] >= semaforoTimers[i][semaforoEstado[i]]):
            semaforoTiempos[i]= 0.0
            if semaforoEstado[i] == 0:
                semaforoEstado[i]=1
            else:
                semaforoEstado[i]=0

"""""""""""""""""""""""""""""""""AGENTE CARRO"""""""""""""""""""""""""""""""""""""""""""""""""""
class CarroAgent(ap.Agent):

    def setup(self):
        self.maxVelocidad = 14.0
        self.minVelocidad = 0.0
        self.maxAceleracion = 10.0
        self.minAceleracion = -30.0
        self.aceleracion = 0.0
        self.velocidad = self.maxVelocidad
        self.direccionVelocidad = [0,0]
        self.rotacionCarro = 0.0
        self.maxRotacionLlantas = 20.0
        self.finished = False
        
              
    def setup_pos(self, space):
        self.neighbors = space.neighbors
        self.neighborList = self.neighbors(self,distance=100).to_list() 
        self.space = space
        self.pos = space.positions[self]
        self.currentNode = 0
        self.currentPathI = 0
        self.checarNodoInicial()
        self.finalNode = nodosFinales[np.random.randint(0,len(nodosFinales))]
        self.nodePath = []
        if(self.currentNode == 0):
            if np.random.randint(0,2) == 0:
                self.nodePath = [0,8,4,10,5,1]
            else:
                self.nodePath = [0,8,4,11,7,3]
            
        if(self.currentNode == 2):
            if np.random.randint(0,2) == 0:
                self.nodePath = [2,9,6,13,5,1]
            else:
                self.nodePath = [2,9,6,12,7,3]
        
        self.nextNode = self.nodePath[self.currentPathI+1]
        self.distanciaNext = 0.0
        self.anguloNext = 0.0
        self.rotacionLlantas = 0.0
        self.inicializarRotacionCarro()
        self.pos[0] -= (self.direccionVelocidad[0] * contadorCarros[nodosIniciales.index(self.currentNode)])
        self.pos[1] -= (self.direccionVelocidad[1] * contadorCarros[nodosIniciales.index(self.currentNode)])
        contadorCarros[nodosIniciales.index(self.currentNode)]+=1;

    
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
    
    def checarNodoInicial(self):
        for i in range(len(grafo)):
            if grafo[i][0] == self.pos[0] and grafo[i][1] == self.pos[1]:
                self.currentNode = i
                
    def inicializarRotacionCarro(self):
        diferenciaY = grafo[self.nextNode][1] - grafo[self.currentNode][1]
        diferenciaX = grafo[self.nextNode][0] - grafo[self.currentNode][0]
        self.distanciaNext=np.sqrt(np.power(diferenciaY,2)+np.power(diferenciaX,2))
        
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

    def formulaRotacionLlanta(self, vx):
        #return (self.maxRotacionLlantas/180)*np.abs(vx)
        return self.maxRotacionLlantas/(1+ np.exp((-1/40)*(vx-90)))
    
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
    
    def checarNextNode(self):
        if(np.sqrt(np.power(grafo[self.nextNode][1] - self.pos[1],2)+np.power(grafo[self.nextNode][0] - self.pos[0],2)) < 1):
            self.currentNode = self.nextNode
            self.currentPathI += 1
            if(self.currentPathI < len(self.nodePath)-1):
                self.nextNode = self.nodePath[self.currentPathI+1]
            else:
                self.velocidad = 0.0
                self.finished=True
                tiemposLlegada.append(tiempo[0])
                self.pos[0]=100.0 + 2*(len(tiemposLlegada))
                self.pos[1]=100.0
                
    
    def calcularRelacionVecinos(self):
        distancia=5.0
        anguloVecino=0.0
        if(len(self.neighborList)>0):
            for nbs in self.neighborList:
                if(np.sqrt(np.power(nbs.pos[1] - self.pos[1],2)+np.power(nbs.pos[0] - self.pos[0],2)) <= distancia):
                    distancia = np.sqrt(np.power(nbs.pos[1] - self.pos[1],2)+np.power(nbs.pos[0] - self.pos[0],2))
                    if(distancia<=0.3 and not self.finished):
                        choques[0]+=1
                        self.finished = True
                        
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
                        
                    elif(self.rotacionCarro < self.anguloNext):
                        if(np.abs(anguloVecino-self.rotacionCarro) < np.abs(anguloVecino-self.rotacionCarro-360)):
                            if(np.abs(anguloVecino-self.rotacionCarro) <= 10):
                                self.aceleracion = self.minAceleracion
                        else:
                            if(np.abs(self.rotacionCarro-anguloVecino-360) <= 10):
                                self.aceleracion = self.minAceleracion

                    else:
                        if(np.abs(self.rotacionCarro-anguloVecino) < np.abs(self.rotacionCarro-anguloVecino-360)):
                            if(np.abs(anguloVecino-self.rotacionCarro) <=10):
                                self.aceleracion = self.minAceleracion
                        
                        else:
                            if(np.abs(self.rotacionCarro-anguloVecino-360) <= 10):
                                self.aceleracion = self.minAceleracion
                    
                    
    def movimiento(self):
        if(not self.finished):
            
            self.aceleracion = self.maxAceleracion
            #Calculo del ángulo del auto
            self.calcularRotacionCarro()
            
            #Cálculo de la rotación de las llantas y frenado para vueltas
            self.calcularRelacionNextNode()
            self.calcularRotacionLlantas()
            
            #Frenado por semáforo
            if(semaforoNodo[self.currentNode]!=-1):
                if(semaforoEstado[semaforoNodo[self.currentNode]]==1):
                    self.aceleracion = self.minAceleracion
            
            self.calcularRelacionVecinos()
            #print(self.aceleracion)
            #Cálculo de velocidad y desplazamiento
            self.velocidad += (self.aceleracion*parameters2D['second_per_step'])
            if(self.velocidad < self.minVelocidad):
                self.velocidad = self.minVelocidad
            if(self.velocidad > self.maxVelocidad):
                self.velocidad = self.maxVelocidad
            
            self.direccionVelocidad[0] = np.cos(np.deg2rad(self.rotacionCarro + self.rotacionLlantas)) * self.velocidad
            self.direccionVelocidad[1] = np.sin(np.deg2rad(self.rotacionCarro + self.rotacionLlantas)) * self.velocidad
            
            self.pos[0] += (self.direccionVelocidad[0] * parameters2D['second_per_step'])
            self.pos[1] += (self.direccionVelocidad[1] * parameters2D['second_per_step'])
            
            #Se checa si cambio de nodo
            self.checarNextNode()
            pass

"""""""""""""""""""""""""""""""""MODELO"""""""""""""""""""""""""""""""""""""""""""""""""""
class MyModel(ap.Model):

    def setup(self):
        self.space = ap.Space(self, shape=[self.p.size]*self.p.ndim)
        self.agents = ap.AgentList(self, self.p.population, CarroAgent)
        self.space.add_agents(self.agents, positions=self.initPosicion())
        self.agents.setup_pos(self.space)

    def step(self):
        tiempo[0] += parameters2D['second_per_step']
        actualizarSemaforos(parameters2D['second_per_step'])
        self.agents.movimiento()
        plt.show()

    def update(self):
        """ Record a dynamic variable. """
        

    def end(self):
        """ Repord an evaluation measure. """
        #self.report('my_measure', 1)
    
    def initPosicion(self):
        res = []
        for i in range(self.p.population):
            res.append(grafo[nodosIniciales[np.random.randint(0,len(nodosIniciales))]])
        return res

"""""""""""""""""""""""""""""""""ANIMACION"""""""""""""""""""""""""""""""""""""""""""""""""""
parameters2D = {
    'size': 40,
    'seed': 123,
    'steps': 500,
    'ndim': 2,
    'population': 8,
    'inner_radius': 3,
    'outer_radius': 10,
    'border_distance': 10,
    'cohesion_strength': 0.005,
    'seperation_strength': 0.1,
    'alignment_strength': 0.3,
    'border_strength': 0.5,
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


animation_plot(MyModel, parameters2D)

if(len(tiemposLlegada)>0):
    print("Primera llegada: "+str(tiemposLlegada[0])+" segundos")
    print("Última llegada: "+str(tiemposLlegada[len(tiemposLlegada)-1])+" segundos")
    print("Tiempo promedio de llegada: "+str(sum(tiemposLlegada)/len(tiemposLlegada))+" segundos")
    print("Cantidad de destinos alcanzados: "+str(len(tiemposLlegada))+"/"+str(parameters2D['population']))
    print("Cantidad de choques: "+str(choques[0]))

"""myFile = open("resultados.csv", "w")
myFile.write("Tiempo primera,Tiempo ultima,Tiempo promedio,Destinos alcanzados,Choques\n")
experimentos=3
for i in range(experimentos):
    model = MyModel(parameters2D)
    results = model.run()
    
    if(len(tiemposLlegada)>0):
        print("Primera llegada: "+str(tiemposLlegada[0])+" segundos")
        print("Última llegada: "+str(tiemposLlegada[len(tiemposLlegada)-1])+" segundos")
        print("Tiempo promedio de llegada: "+str(sum(tiemposLlegada)/len(tiemposLlegada))+" segundos")
        print("Cantidad de destinos alcanzados: "+str(len(tiemposLlegada))+"/"+str(parameters2D['population']))
        print("Cantidad de choques: "+str(choques[0]))
        print()
        
        writeContent=""
        
        totalTiempoPrimera.append(tiemposLlegada[0])
        writeContent = str(writeContent) + str(tiemposLlegada[0]) + ","
        
        totalTiempoUltima.append(tiemposLlegada[len(tiemposLlegada)-1])
        writeContent = str(writeContent) + str(tiemposLlegada[len(tiemposLlegada)-1]) + ","
        
        totalTiempoPromedio.append(sum(tiemposLlegada))
        writeContent = str(writeContent) + str(sum(tiemposLlegada)/len(tiemposLlegada)) + ","
        
        totalDestinos.append(len(tiemposLlegada))
        writeContent = str(writeContent) + str(float(len(tiemposLlegada)/parameters2D['population'])) + ","
        
        totalChoques.append(choques[0])
        writeContent = str(writeContent) + str(choques[0]) + "\n"
        
        myFile.write(writeContent)
        
    tiempo = [0.0]
    tiemposLlegada = []
    choques = [0]

writeContent = str(sum(totalTiempoPrimera)/len(totalTiempoPrimera))+","
writeContent = writeContent+str(sum(totalTiempoUltima)/len(totalTiempoUltima))+","
writeContent = writeContent+str(sum(totalTiempoPromedio)/sum(totalDestinos))+","
writeContent = writeContent+str(sum(totalDestinos)/(parameters2D['population']*experimentos))+","
writeContent = writeContent+str(sum(totalChoques)/(parameters2D['population']*experimentos))+"\n" 
myFile.write("\n")
myFile.write(writeContent)
myFile.close()   
if(len(totalTiempoPromedio)>0):
    print("Total promedio - Primera llegada: "+str(sum(totalTiempoPrimera)/len(totalTiempoPrimera))+" segundos")
    print("Total promedio - Última llegada: "+str(sum(totalTiempoUltima)/len(totalTiempoUltima))+" segundos")
    print("Total promedio - Tiempo promedio de llegada: "+str(sum(totalTiempoPromedio)/sum(totalDestinos))+" segundos")
    print("Total promedio - Cantidad de destinos alcanzados: "+str(float(sum(totalDestinos)/(parameters2D['population']*experimentos))*100)+"%")
    print("Total promedio - Cantidad de choques: "+str(sum(totalChoques)/(parameters2D['population']*experimentos))+"%")
    print()"""
