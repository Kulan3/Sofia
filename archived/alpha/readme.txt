to setup --> setup.sh


Fly newest plan, AI OFF (ignores config):

python \main.py --use-last --ai off


Fly specific plan with policy 1 (resume when lost):

python \main.py waypoint.json --ai on --mode 1


Fly newest plan with policy 2 (approach + hold), use config logging path auto:

python .\main.py --use-last --ai auto --mode 2 --log=""