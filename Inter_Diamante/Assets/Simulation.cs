using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class Simulation : MonoBehaviour
{
    public SimulationParametros parametros;
    private List<Vector3> positions;
    private List<Vector3> rotations;
    private int currentStep = 0;
    public int x;
    public TextAsset csvFile;
    public GameObject[] carModels;
    public GameObject[] cars;
    private bool firstTime = true;
    private int r;

    // Start is called before the first frame update
    void Start()
    {
        //Application.targetFrameRate = parametros.frameRate;
        
        ReadCSVFile();
    }

    void ReadCSVFile()
    {
        positions = new List<Vector3>();
        rotations = new List<Vector3>();
        string[] data = csvFile.text.Split('\n');
        for(int i = 0; i < data.Length; i++)
        {
            string[] dataValues = data[i].Split(',');
            positions.Add(new Vector3(float.Parse(dataValues[0]), float.Parse(dataValues[2]), float.Parse(dataValues[1])));
            rotations.Add(new Vector3(0f, -1*float.Parse(dataValues[3]), float.Parse(dataValues[4])));
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(firstTime)
        {
            x = parametros.cantidadCarros;
            for(int i = 0; i < x; i++)
            {
                r = Random.Range(0,4);
                Instantiate(carModels[r], new Vector3(0f,0f,0f), Quaternion.identity);
            }
            cars = GameObject.FindGameObjectsWithTag("Car");
            firstTime = false;
        }
        for(int i = 0; i < x; i++)
        {
            cars[i].transform.position = positions[currentStep+i];
            cars[i].transform.eulerAngles = rotations[currentStep+i];
        }
        currentStep = currentStep + x;
    }
}
