def finanzierungssystem():
    vollpreis = 0
    anzahlung = 0
    finanzierungsdauer = 0
    while(vollpreis <= 0):
        vollpreis = float(input("Geben Sie den Vollpreis des Wunschautos ein: "))
        if vollpreis <= 0:
            print("Der Vollpreis muss größer als 0 sein. Bitte versuchen Sie es erneut.")
    while(anzahlung <= 0):
        anzahlung = float(input("Geben Sie die Anzahlung ein: "))
        if anzahlung <= 0:
            print("Die Anzahlung muss größer 0 sein. Bitte versuchen Sie es erneut.")
    while(finanzierungsdauer not in (6, 12, 24, 48)):
        finanzierungsdauer = int(input("Geben Sie die Finanzierungsdauer in Monaten ein (6, 12, 24 oder 48): "))
        if finanzierungsdauer not in (6, 12, 24, 48):
            print("Die Finanzierungsdauer muss entweder 6, 12, 24 oder 48 Monate betragen. Bitte versuchen Sie es erneut.")
            
    if finanzierungsdauer == 6:
        zinssatz = 0.02
    elif finanzierungsdauer == 12:
        zinssatz = 0.04
    elif finanzierungsdauer == 24:
        zinssatz = 0.06
    elif finanzierungsdauer == 48:
        zinssatz = 0.08
        
    print("Der Zinssatz für die gewählte Finanzierungsdauer beträgt: ", zinssatz * 100, "%")
    print("Die monatliche Rate beträgt: ", (vollpreis - anzahlung) * (1 + zinssatz) / finanzierungsdauer)
    
finanzierungssystem()