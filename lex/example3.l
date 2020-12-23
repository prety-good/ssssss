%{

int your_select,total=0;

%}



%%
dd[d|q]		 	       { printf("请选糖: 1:巧克力,2:太妃糖\n"); 
								scanf("%d",&your_select);
								if(your_select==1) printf("你选了巧克力\n");
								if(your_select==2) printf("你选了太妃糖\n");
								total++;
				}
dq				{ printf("请选糖: 1:巧克力,2:太妃糖\n"); scanf("%d",&your_select);
								if(your_select==1) printf("你选了巧克力\n");
								if(your_select==2) printf("你选了太妃糖\n");
								total++;
				}
q				{ printf("请选糖: 1:巧克力,2:太妃糖\n"); scanf("%d",&your_select);
								if(your_select==1) printf("你选了巧克力\n");
								if(your_select==2) printf("你选了太妃糖\n");
								total++;
				} 
.				{ printf("投币过多或过少或非法货币\n");} /* ignore everything else */


%%
main()
{
	yylex();
	printf("共购买了%d糖果\n",total); 
}

int yywrap()
{return 1;}
