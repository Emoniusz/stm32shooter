# O projekcie
Jest to prosta gra którą przygotowałem na zajęciach na uczelni. Została ona opracowana dla płytki rozwojowej STM32F429I-DISCO. Rozgrywka polega na strzelaniu do pojawiających się co rundę losowych postaci, dobrych i złych. Gracz ma ograniczony czas na podjęcie decyzji, do której postaci chce strzelić. Zależnie od wyboru gracz może zdobyć lub stracić punkty. Za otrzymanie odpowiedniej ilości punktów liczba żyć zwiększa się o 1. Gdy czas na podjęcie decyzji się zakończy, złe postacie które nie zostały zastrzelone strzelają do gracza przez co zmniejsza się ilość pozostałych żyć. Jeżeli liczba żyć zmniejszy się do 0, gra się zatrzymuje, należy ją wtedy zrestartować. W innym przypadku rozpoczyna się kolejna runda.
# Wykorzystane komponenty
Do projektu wykorzystałem jedynie  STM32F429I-DISCO oraz 3 przyciski. Były one kolejno podłączone do pinów PE2, PE4 i PE6 (oraz do 3V).  Każdy z nich służy do strzelania do innej postaci. 
# Wyświetlanie zdjęć
Najwięcej czasu przy tym projekcie poświęciłem na wymyślenie sposobu, jak wyświetlić zdjęcia na wyświetlaczu. Wykorzystałem w tym celu funkcję BSP_LCD_DrawBitmap() z sterownika wyświetlacza stm32f429i_discovery_lcd.c. Wcześniej przygotowane zdjęcia przekonwertowałem na tablice w języku C i dołączyłem je do projektu.
# Wykrywanie strzału 
Do wykrywania strzału wykorzystałem mechanizm przerwań. Każdy z trzech przycisków przerywa główną pętlę programu i wywołuje mechanizm obsługi przerwań. Jeżeli program jest w momencie kiedy strzelanie jest dozwolone zmienia obrazki oraz punkty. W innym przypadku strzał jest ignorowany.
