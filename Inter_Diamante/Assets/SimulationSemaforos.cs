using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SimulationSemaforos : MonoBehaviour
{
    public SimulationParametros parametros;
    private int currentStep = 0;
    public int x;
    private List<int> estados;
    public TextAsset csvSemaforos;
    public GameObject semaforo;
    public GameObject[] semaforos;
    private bool firstTime = true;
    public Material verde;
    public Material rojo;
    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = parametros.frameRate;
        x = parametros.cantidadSemaforos;
        ReadCSVFile();
    }

    void ReadCSVFile()
    {
        estados = new List<int>();
        string[] data = csvSemaforos.text.Split('\n');
        for(int i = 0; i < data.Length; i++)
        {
            estados.Add(int.Parse(data[i]));
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(firstTime)
        {
            for(int i = 0; i < x; i++)
            {
                Instantiate(semaforo, parametros.semaforoPositions[i], Quaternion.identity);
            }
            semaforos = GameObject.FindGameObjectsWithTag("Semaforo");
            for(int i = 0; i < x; i++)
            {
                //semaforos[i].transform.position = parametros.semaforoPositions[i];
            }
            firstTime = false;
        }
        for(int i = 0; i < x; i++)
        {
            if(estados[currentStep+i] == 1)
            {
                semaforos[i].GetComponent<MeshRenderer>().material = rojo;
            }
            else{
                semaforos[i].GetComponent<MeshRenderer>().material = verde;
            }
        }
        currentStep = currentStep + x;
    }
}
